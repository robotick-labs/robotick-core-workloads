// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#pragma once
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
		float envelope_lp_hz = 30.0f;
		float compression_gamma = 1.0f;
		float mod_low_hz = 2.0f;
		float mod_high_hz = 20.0f;
		float erb_bandwidth_scale = 0.4f; // <— scales ERB width (0.5 = half normal)
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
		double frame_rate = 0.0;

		FixedVector<float, FrameSize> window;
		FixedVector<float, FrameSize> fft_in;
		FixedVector<float, FFTBins> mag;
		FixedVector<float, FFTBins> phase;
		FixedVector<kiss_fft_cpx, FFTBins> fft_out;

		FixedVector<BandInfo, AudioBuffer128::capacity()> bands;

		float env_alpha = 0.0f;
		AudioBuffer128 env_prev;
		float mod_hp_a0 = 0.0f, mod_hp_b1 = 0.0f, mod_hp_c1 = 0.0f;
		float mod_lp_a0 = 0.0f, mod_lp_b1 = 0.0f, mod_lp_c1 = 0.0f;
		AudioBuffer128 mod_hp_z1;
		AudioBuffer128 mod_lp_z1;

		kiss_fftr_cfg cfg_fftr = nullptr;
		alignas(16) unsigned char kiss_cfg_mem[131072]{};

		void build_window()
		{
			window.set_size(FrameSize);
			const float N = float(FrameSize);
			for (size_t n = 0; n < FrameSize; ++n)
				window[n] = 0.5f * (1.0f - std::cos(2.0f * float(M_PI) * float(n) / (N - 1.0f)));
		}

		void plan_fft()
		{
			fft_in.fill(0.0f);
			size_t len = sizeof(kiss_cfg_mem);
			cfg_fftr = kiss_fftr_alloc(int(FFTSize), 0, kiss_cfg_mem, &len);
			mag.set_size(FFTBins);
			phase.set_size(FFTBins);
			fft_out.set_size(FFTBins);
		}

		static inline float erb_rate(float hz) { return 21.4f * std::log10(4.37e-3f * hz + 1.0f); }
		static inline float inv_erb_rate(float erb) { return (std::pow(10.0f, erb / 21.4f) - 1.0f) / 4.37e-3f; }

		static inline int hz_to_bin(float hz, uint32_t sr)
		{
			const float bin_hz = float(sr) / float(FFTSize);
			int k = int(std::round(hz / bin_hz));
			return clamp_bin(k);
		}

		static inline int clamp_bin(int k)
		{
			if (k < 0)
				return 0;
			if (k >= int(FFTBins))
				return int(FFTBins - 1);
			return k;
		}

		void build_erb_bands(const CochlearTransformConfig& cfg)
		{
			bands.set_size(cfg.num_bands);

			const float e0 = erb_rate(cfg.fmin_hz);
			const float e1 = erb_rate(cfg.fmax_hz);
			const float step = (cfg.num_bands > 1) ? ((e1 - e0) / float(cfg.num_bands - 1)) : 0.0f;

			for (uint16_t bandId = 0; bandId < cfg.num_bands; ++bandId)
			{
				const float e = e0 + step * float(bandId);
				const float fc = inv_erb_rate(e);
				auto& band = bands[bandId];
				band.center_hz = fc;

				// narrower ERB width for sharper response
				const float bw = cfg.erb_bandwidth_scale * 24.7f * (4.37e-3f * fc + 1.0f);
				const float left_hz = std::max(cfg.fmin_hz, fc - bw);
				const float right_hz = std::min(cfg.fmax_hz, fc + bw);

				band.left_bin = hz_to_bin(left_hz, sample_rate);
				band.center_bin = hz_to_bin(fc, sample_rate);
				band.right_bin = hz_to_bin(right_hz, sample_rate);
				if (band.right_bin <= band.center_bin)
					band.right_bin = band.center_bin + 1;
			}
		}

		void build_env_filters(const CochlearTransformConfig& cfg)
		{
			const double dt = 1.0 / frame_rate;
			const double tau = (cfg.envelope_lp_hz <= 0.0f) ? 0.0 : 1.0 / (2.0 * M_PI * double(cfg.envelope_lp_hz));
			env_alpha = (tau <= 0.0) ? 1.0f : float(1.0 - std::exp(-dt / tau));

			const double fs = frame_rate;
			{
				const double fc = std::max(0.1, double(cfg.mod_low_hz));
				const double x = std::exp(-2.0 * M_PI * fc / fs);
				mod_hp_a0 = float((1.0 + x) * 0.5);
				mod_hp_b1 = float(x);
				mod_hp_c1 = float(x);
			}
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
			env_prev.fill(0.0f);
			mod_hp_z1.fill(0.0f);
			mod_lp_z1.fill(0.0f);
		}
	};

	// =====================================================================
	struct CochlearTransformWorkload
	{
		CochlearTransformConfig config;
		CochlearTransformInputs inputs;
		CochlearTransformOutputs outputs;
		State<CochlearTransformState> state;

		void load()
		{
			AudioSystem::init();
			state->sample_rate = AudioSystem::get_sample_rate();

			// Hop = 1/4 window for 75% overlap
			const double hop = CochlearTransformState::FrameSize / 4.0;
			state->frame_rate = double(state->sample_rate) / hop;

			const size_t cap = AudioBuffer128::capacity();
			config.num_bands = std::min(cap, (size_t)config.num_bands);

			outputs.cochlear_frame.envelope.set_size(config.num_bands);
			outputs.cochlear_frame.fine_phase.set_size(config.num_bands);
			outputs.cochlear_frame.modulation_power.set_size(config.num_bands);

			state->build_window();
			state->plan_fft();
			state->build_erb_bands(config);
			state->build_env_filters(config);
			state->reset_state();
		}

		void tick(const TickInfo&)
		{
			const size_t N = std::min(inputs.mono.samples.size(), CochlearTransformState::FrameSize);
			for (size_t i = 0; i < N; ++i)
			{
				state->fft_in[i] = inputs.mono.samples[i] * state->window[i];
			}
			for (size_t i = N; i < CochlearTransformState::FrameSize; ++i)
			{
				state->fft_in[i] = 0.0f; // zero-pad remainder
			}

			kiss_fftr(state->cfg_fftr, state->fft_in.data(), state->fft_out.data());

			for (size_t k = 0; k < CochlearTransformState::FFTBins; ++k)
			{
				const float re = state->fft_out[k].r;
				const float im = state->fft_out[k].i;
				state->mag[k] = std::sqrt(re * re + im * im) + 1e-12f;
				state->phase[k] = std::atan2(im, re);
			}

			for (size_t bandId = 0; bandId < config.num_bands; ++bandId)
			{
				const auto& band = state->bands[bandId];
				const float fc = band.center_hz;
				const float bw = config.erb_bandwidth_scale * 24.7f * (4.37e-3f * fc + 1.0f);

				float energy = 0.0f;
				float wsum = 0.0f;

				for (int k = band.left_bin; k < band.right_bin; ++k)
				{
					const float bin_hz = (float(k) / CochlearTransformState::FFTSize) * state->sample_rate;
					const float w = std::exp(-0.5f * std::pow((bin_hz - fc) / (0.5f * bw), 2.0f)); // Gaussian weight
					const float m = state->mag[k];
					energy += w * (m * m);
					wsum += w;
				}

				if (wsum > 0.0f)
					energy /= wsum; // normalise by total weight

				const float amp = std::sqrt(energy);
				const float env = state->env_alpha * amp + (1.0f - state->env_alpha) * state->env_prev[bandId];
				state->env_prev[bandId] = env;

				const float hp_y = state->mod_hp_a0 * env + state->mod_hp_b1 * state->mod_hp_z1[bandId];
				state->mod_hp_z1[bandId] = env - state->mod_hp_c1 * hp_y;

				const float lp_y = state->mod_lp_a0 * hp_y + state->mod_lp_b1 * state->mod_lp_z1[bandId];
				state->mod_lp_z1[bandId] = hp_y - state->mod_lp_c1 * lp_y;

				outputs.cochlear_frame.envelope[bandId] = env;
				outputs.cochlear_frame.modulation_power[bandId] = lp_y * lp_y;
				outputs.cochlear_frame.fine_phase[bandId] = state->phase[band.center_bin];
			}

			outputs.cochlear_frame.timestamp = inputs.mono.timestamp;
		}

		void stop() {}
	};
} // namespace robotick
