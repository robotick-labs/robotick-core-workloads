// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#include "robotick/api.h"
#include "robotick/systems/audio/AudioFrame.h"
#include "robotick/systems/audio/AudioSystem.h"
#include "robotick/systems/auditory/CochlearFrame.h"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <kissfft/kiss_fftr.h>

namespace robotick
{
	struct CochlearTransformConfig
	{
		uint16_t num_bands = 128;
		float fmin_hz = 50.0f;
		float fmax_hz = 3500.0f;
		float envelope_lp_hz = 30.0f;	  // envelope smoothing (Hz)
		float compression_gamma = 0.6f;	  // perceptual dynamic compression
		float mod_low_hz = 1.5f;		  // modulation HP (on envelope)
		float mod_high_hz = 12.0f;		  // modulation LP (on envelope)
		float erb_bandwidth_scale = 0.5f; // scales ERB width (narrower => sharper)
		bool use_preemphasis = true;
		float preemph = 0.97f; // preemphasis factor
	};

	struct CochlearTransformInputs
	{
		AudioFrame mono;
	};

	struct CochlearTransformOutputs
	{
		CochlearFrame cochlear_frame;
	};

	// =====================================================================
	struct CochlearTransformState
	{
		static constexpr size_t FrameSize = 4096;
		static constexpr size_t Hop = FrameSize / 4; // 75% overlap
		static constexpr size_t FFTSize = FrameSize;
		static constexpr size_t FFTBins = FFTSize / 2 + 1;

		struct BandInfo
		{
			float center_hz = 0.0f;
			int left_bin = 0;
			int center_bin = 0;
			int right_bin = 0;
		};

		uint32_t sample_rate = 44100;
		double frame_rate = 0.0; // sample_rate / Hop

		// Windowed STFT working buffers
		FixedVector<float, FrameSize> window;
		FixedVector<float, FrameSize> fft_in;
		FixedVector<float, FFTBins> mag;
		FixedVector<float, FFTBins> phase;
		FixedVector<kiss_fft_cpx, FFTBins> fft_out;

		// Persistent circular buffer for true sliding windows
		FixedVector<float, FrameSize> ring;
		size_t write_idx = 0;
		size_t filled = 0;
		size_t samples_since_last_frame = 0;

		FixedVector<BandInfo, AudioBuffer128::capacity()> bands;

		// Envelope + modulation filters (per band)
		float env_alpha = 0.0f;
		AudioBuffer128 env_prev;
		float mod_hp_a0 = 0.0f, mod_hp_b1 = 0.0f, mod_hp_c1 = 0.0f;
		float mod_lp_a0 = 0.0f, mod_lp_b1 = 0.0f, mod_lp_c1 = 0.0f;
		AudioBuffer128 mod_hp_z1;
		AudioBuffer128 mod_lp_z1;

		// Preemphasis + DC removal
		float x_prev = 0.0f;
		float dc_state = 0.0f;
		float dc_alpha = 0.9995f;

		kiss_fftr_cfg cfg_fftr = nullptr;
		alignas(16) unsigned char kiss_cfg_mem[131072]{};

		float window_rms = 1.0f;

		// ---------------- Window/FFT planning ----------------
		void build_window()
		{
			window.set_size(FrameSize);
			const float N = float(FrameSize);
			double e = 0.0;
			for (size_t n = 0; n < FrameSize; ++n)
			{
				const float w = 0.5f * (1.0f - std::cos(2.0f * float(M_PI) * (float)n / (N - 1.0f))); // Hann
				window[n] = w;
				e += double(w) * double(w);
			}
			window_rms = (e > 0.0) ? float(std::sqrt(e / double(FrameSize))) : 1.0f;
		}

		void plan_fft()
		{
			fft_in.set_size(FrameSize);
			fft_in.fill(0.0f);

			size_t len = sizeof(kiss_cfg_mem);
			cfg_fftr = kiss_fftr_alloc(int(FFTSize), 0, kiss_cfg_mem, &len);
			if (!cfg_fftr)
				cfg_fftr = kiss_fftr_alloc(int(FFTSize), 0, nullptr, nullptr);
			ROBOTICK_ASSERT(cfg_fftr && "kiss_fftr_alloc failed");

			mag.set_size(FFTBins);
			phase.set_size(FFTBins);
			fft_out.set_size(FFTBins);
		}

		// ---------------- ERB helpers ----------------
		static inline float erb_rate(float hz) { return 21.4f * std::log10(4.37e-3f * hz + 1.0f); }
		static inline float inv_erb_rate(float erb) { return (std::pow(10.0f, erb / 21.4f) - 1.0f) / 4.37e-3f; }

		static inline int clamp_bin(int k)
		{
			if (k < 0)
				return 0;
			const int maxk = int(FFTBins) - 1;
			if (k > maxk)
				return maxk;
			return k;
		}

		static inline int hz_to_bin(float hz, uint32_t sr)
		{
			const float bin_hz = float(sr) / float(FFTSize);
			return clamp_bin(int(std::round(hz / bin_hz)));
		}

		void build_erb_bands(const CochlearTransformConfig& cfg)
		{
			bands.set_size(cfg.num_bands);

			const float e0 = erb_rate(cfg.fmin_hz);
			const float e1 = erb_rate(cfg.fmax_hz);
			const float step = (cfg.num_bands > 1) ? ((e1 - e0) / float(cfg.num_bands - 1)) : 0.0f;

			for (uint16_t b = 0; b < cfg.num_bands; ++b)
			{
				const float e = e0 + step * float(b);
				const float fc = inv_erb_rate(e);
				auto& band = bands[b];
				band.center_hz = fc;

				const float bw = cfg.erb_bandwidth_scale * 24.7f * (4.37e-3f * fc + 1.0f);
				const float left_hz = std::max(cfg.fmin_hz, fc - bw);
				const float right_hz = std::min(cfg.fmax_hz, fc + bw);

				band.left_bin = hz_to_bin(left_hz, sample_rate);
				band.center_bin = hz_to_bin(fc, sample_rate);
				band.right_bin = hz_to_bin(right_hz, sample_rate);

				if (band.right_bin <= band.left_bin)
					band.right_bin = std::min(int(FFTBins - 1), band.left_bin + 1);
				if (band.center_bin <= band.left_bin)
					band.center_bin = std::min(band.right_bin - 1, band.left_bin + (band.right_bin - band.left_bin) / 2);
			}
		}

		// ---------------- Envelope & Modulation filters ----------------
		void build_env_filters(const CochlearTransformConfig& cfg)
		{
			const double dt = 1.0 / frame_rate;
			const double fc_env = std::clamp(double(cfg.envelope_lp_hz), 0.5, 60.0);
			const double tau = 1.0 / (2.0 * M_PI * fc_env);
			env_alpha = float(1.0 - std::exp(-dt / tau));

			const double fs = frame_rate;
			// Simple first-order HP on envelope
			{
				const double fc = std::max(0.1, double(cfg.mod_low_hz));
				const double x = std::exp(-2.0 * M_PI * fc / fs);
				mod_hp_a0 = float((1.0 + x) * 0.5);
				mod_hp_b1 = float(x);
				mod_hp_c1 = float(x);
			}
			// Simple first-order LP after HP (band-limit envelope modulation)
			{
				const double fc = std::max(0.1, double(cfg.mod_high_hz));
				const double x = std::exp(-2.0 * M_PI * fc / fs);
				mod_lp_a0 = float(1.0 - x);
				mod_lp_b1 = float(x);
				mod_lp_c1 = float(x);
			}
		}

		void reset_state()
		{
			ring.set_size(FrameSize);
			ring.fill(0.0f);
			write_idx = 0;
			filled = 0;
			samples_since_last_frame = 0;

			env_prev.fill(0.0f);
			mod_hp_z1.fill(0.0f);
			mod_lp_z1.fill(0.0f);
			x_prev = 0.0f;
			dc_state = 0.0f;
		}

		// ---------------- Ring buffer intake ----------------
		void push_samples(const float* src, size_t n, const CochlearTransformConfig& cfg)
		{
			if (!src || n == 0)
				return;
			for (size_t i = 0; i < n; ++i)
			{
				float x = src[i];
				// DC tracker
				dc_state = dc_alpha * dc_state + (1.0f - dc_alpha) * x;
				x -= dc_state;
				// Preemphasis
				if (cfg.use_preemphasis)
				{
					const float y = x - x_prev * cfg.preemph;
					x_prev = x;
					x = y;
				}
				ring[write_idx] = x;
				write_idx = (write_idx + 1) % FrameSize;
				if (filled < FrameSize)
					++filled;
				++samples_since_last_frame;
			}
		}

		bool make_frame_from_ring()
		{
			if (filled < FrameSize || samples_since_last_frame < Hop)
				return false;

			size_t r = write_idx;
			for (size_t i = 0; i < FrameSize; ++i)
			{
				fft_in[i] = (ring[r] * window[i]) / window_rms;
				r = (r + 1) % FrameSize;
			}
			samples_since_last_frame -= Hop;
			return true;
		}
	};

	// =====================================================================
	struct CochlearTransformWorkload
	{
		CochlearTransformConfig config;
		CochlearTransformInputs inputs;
		CochlearTransformOutputs outputs;
		State<CochlearTransformState> state;

		static inline float zap_denorm(float v) { return (std::fabs(v) < 1e-30f) ? 0.0f : v; }

		void load()
		{
			AudioSystem::init();
			state->sample_rate = AudioSystem::get_sample_rate();

			state->build_window();
			state->plan_fft();

			state->frame_rate = double(state->sample_rate) / double(CochlearTransformState::Hop);

			const size_t cap = AudioBuffer128::capacity();
			if (config.num_bands > cap)
				config.num_bands = (uint16_t)cap;

			outputs.cochlear_frame.envelope.set_size(config.num_bands);
			outputs.cochlear_frame.fine_phase.set_size(config.num_bands);
			outputs.cochlear_frame.modulation_power.set_size(config.num_bands);
			outputs.cochlear_frame.band_center_hz.set_size(config.num_bands);

			state->build_erb_bands(config);
			state->build_env_filters(config);
			state->reset_state();
		}

		void analyze_one_frame()
		{
			kiss_fftr(state->cfg_fftr, state->fft_in.data(), state->fft_out.data());

			for (size_t k = 0; k < CochlearTransformState::FFTBins; ++k)
			{
				const float re = state->fft_out[k].r;
				const float im = state->fft_out[k].i;
				const float p = std::sqrt(re * re + im * im);
				state->mag[k] = p + 1e-12f;
				state->phase[k] = std::atan2(im, re);
			}

			// light 3-tap blur to smooth across frequency
			for (size_t k = 1; k + 1 < CochlearTransformState::FFTBins; ++k)
			{
				const float m0 = state->mag[k - 1], m1 = state->mag[k], m2 = state->mag[k + 1];
				state->mag[k] = (m0 + 2.0f * m1 + m2) * 0.25f;
			}

			const float bin_hz = float(state->sample_rate) / float(CochlearTransformState::FFTSize);

			for (size_t bandId = 0; bandId < config.num_bands; ++bandId)
			{
				const auto& band = state->bands[bandId];
				const float fc = band.center_hz;
				const float bw = config.erb_bandwidth_scale * 24.7f * (4.37e-3f * fc + 1.0f);

				float energy = 0.0f;
				float wsum = 0.0f;

				for (int k = band.left_bin; k < band.right_bin; ++k)
				{
					const float f_k = bin_hz * float(k);
					const float g = (f_k - fc) / (0.5f * bw);
					const float w = std::exp(-0.5f * g * g);
					const float m = state->mag[k];
					energy += w * (m * m);
					wsum += w;
				}
				if (wsum > 0.0f)
					energy /= wsum;

				const float amp = std::sqrt(energy);
				const float env = state->env_alpha * amp + (1.0f - state->env_alpha) * state->env_prev[bandId];
				state->env_prev[bandId] = env;

				const float comp = std::pow(std::max(env, 0.0f) + 1e-9f, config.compression_gamma);

				float hp_y = state->mod_hp_a0 * comp + state->mod_hp_b1 * state->mod_hp_z1[bandId];
				hp_y = zap_denorm(hp_y);
				state->mod_hp_z1[bandId] = comp - state->mod_hp_c1 * hp_y;

				float lp_y = state->mod_lp_a0 * hp_y + state->mod_lp_b1 * state->mod_lp_z1[bandId];
				lp_y = zap_denorm(lp_y);
				state->mod_lp_z1[bandId] = hp_y - state->mod_lp_c1 * lp_y;

				const float env_vis = comp;

				outputs.cochlear_frame.envelope[bandId] = env_vis;
				outputs.cochlear_frame.modulation_power[bandId] = lp_y * lp_y;
				outputs.cochlear_frame.fine_phase[bandId] = state->phase[band.center_bin];
			}

			for (size_t b = 0; b < config.num_bands; ++b)
			{
				outputs.cochlear_frame.band_center_hz[b] = state->bands[b].center_hz;
			}
		}

		void tick(const TickInfo&)
		{
			if (!inputs.mono.samples.empty())
				state->push_samples(inputs.mono.samples.data(), inputs.mono.samples.size(), config);

			if (!state->make_frame_from_ring())
			{
				outputs.cochlear_frame.timestamp = inputs.mono.timestamp;
				return;
			}

			analyze_one_frame();
			outputs.cochlear_frame.timestamp = inputs.mono.timestamp;
		}

		void stop() {}
	};
} // namespace robotick
