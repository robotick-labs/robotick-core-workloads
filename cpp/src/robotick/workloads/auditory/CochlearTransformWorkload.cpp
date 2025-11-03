// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#include "robotick/api.h"
#include "robotick/systems/audio/AudioFrame.h"
#include "robotick/systems/audio/AudioSystem.h"
#include "robotick/systems/auditory/CochlearFrame.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <kissfft/kiss_fftr.h>

namespace robotick
{
	struct CochlearTransformConfig
	{
		uint16_t num_bands = 128; // Max = AudioBuffer128::capacity()
		float fmin_hz = 50.0f;
		float fmax_hz = 8000.0f;
		float envelope_lp_hz = 30.0f;
		float compression_gamma = 0.3f;
		float mod_low_hz = 2.0f;
		float mod_high_hz = 20.0f;
		bool output_phase = true;
	};

	struct CochlearTransformInputs
	{
		AudioFrame mono;
	};

	struct CochlearTransformOutputs
	{
		CochlearFrame cochlear_frame;
	};

	struct CochlearTransformState
	{
		static constexpr size_t FrameSize = AudioBuffer512::capacity();
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
		double frame_rate = 86.13;

		AudioBuffer512 window;
		AudioBuffer512 fft_in;
		std::array<kiss_fft_cpx, FFTBins> fft_out{};
		AudioBuffer512 mag;
		AudioBuffer512 phase; // first 257 bins used
		std::array<BandInfo, AudioBuffer128::capacity()> bands{};

		float env_alpha = 0.0f;
		AudioBuffer128 env_prev;

		float mod_hp_a0 = 0.0f, mod_hp_b1 = 0.0f, mod_hp_c1 = 0.0f;
		float mod_lp_a0 = 0.0f, mod_lp_b1 = 0.0f, mod_lp_c1 = 0.0f;
		AudioBuffer128 mod_hp_z1;
		AudioBuffer128 mod_lp_z1;

		kiss_fftr_cfg cfg_fftr = nullptr;
		alignas(16) std::array<unsigned char, 8192> kiss_cfg_mem{};

		void build_window()
		{
			window.clear();
			window.set_size(FrameSize);
			const float N = float(FrameSize);
			for (size_t n = 0; n < FrameSize; ++n)
				window[n] = 0.5f * (1.0f - std::cos(2.0f * float(M_PI) * float(n) / (N - 1.0f)));
		}

		void plan_fft()
		{
			size_t len = kiss_cfg_mem.size();
			cfg_fftr = kiss_fftr_alloc(int(FFTSize), 0, kiss_cfg_mem.data(), &len);
		}

		static inline float erb_rate(float hz) { return 21.4f * std::log10(4.37e-3f * hz + 1.0f); }
		static inline float inv_erb_rate(float erb) { return (std::pow(10.0f, erb / 21.4f) - 1.0f) / 4.37e-3f; }

		static inline float hz_to_bin(float hz, uint32_t sr)
		{
			const float bin_hz = float(sr) / float(FFTSize);
			float kf = hz / bin_hz;
			int k = int(std::round(kf));
			if (k < 0)
				k = 0;
			if (k > int(FFTBins - 1))
				k = int(FFTBins - 1);
			return float(k);
		}

		static inline int clamp_bin(int k)
		{
			if (k < 0)
				return 0;
			const int maxk = int(FFTBins) - 1;
			if (k > maxk)
				return maxk;
			return k;
		}

		void build_erb_bands(const CochlearTransformConfig& cfg)
		{
			const float fmin = cfg.fmin_hz;
			const float fmax = cfg.fmax_hz;
			const float e0 = erb_rate(fmin);
			const float e1 = erb_rate(fmax);
			const float step = (cfg.num_bands > 1) ? ((e1 - e0) / float(cfg.num_bands - 1)) : 0.0f;

			for (uint16_t b = 0; b < cfg.num_bands; ++b)
			{
				const float e = e0 + step * float(b);
				const float fc = inv_erb_rate(e);
				bands[b].center_hz = fc;

				const float bw = 24.7f * (4.37e-3f * fc + 1.0f);
				const float left_hz = std::max(fmin, fc - bw);
				const float right_hz = std::min(fmax, fc + bw);

				const int left_k = int(std::floor(hz_to_bin(left_hz, sample_rate) + 0.5f));
				const int center_k = int(std::floor(hz_to_bin(fc, sample_rate) + 0.5f));
				const int right_k = int(std::floor(hz_to_bin(right_hz, sample_rate) + 0.5f));

				bands[b].left_bin = clamp_bin(left_k);
				bands[b].center_bin = clamp_bin(center_k);
				bands[b].right_bin = clamp_bin(right_k > bands[b].center_bin ? right_k : bands[b].center_bin + 1);
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
			env_prev.fill(0.0f);
			mod_hp_z1.fill(0.0f);
			mod_hp_z1.fill(0.0f);
			mod_lp_z1.fill(0.0f);
			mod_lp_z1.fill(0.0f);
		}
	};

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
			state->frame_rate = double(state->sample_rate) / double(CochlearTransformState::FrameSize);

			const size_t capacity = AudioBuffer128::capacity();
			config.num_bands = std::min(capacity, (size_t)config.num_bands);

			state->window.fill(0.0f);
			state->fft_in.fill(0.0f);
			state->mag.fill(0.0f);
			state->phase.fill(0.0f);
			state->env_prev.fill(0.0f);
			state->mod_hp_z1.fill(0.0f);
			state->mod_lp_z1.fill(0.0f);

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
				float energy = 0.0f;

				const int k0 = band.left_bin;
				const int k1 = band.center_bin;
				const int k2 = band.right_bin;

				const float denomL = float(k1 - k0 > 0 ? (k1 - k0) : 1);
				for (int k = k0; k < k1; ++k)
				{
					const float w = (float(k) - float(k0)) / denomL;
					const float m = state->mag[k];
					energy += (w * m) * (w * m);
				}

				const float denomR = float(k2 - k1 > 0 ? (k2 - k1) : 1);
				for (int k = k1; k <= k2; ++k)
				{
					const float w = 1.0f - (float(k) - float(k1)) / denomR;
					const float m = state->mag[k];
					energy += (w * m) * (w * m);
				}

				const float amp = std::sqrt(energy);
				const float env = state->env_alpha * amp + (1.0f - state->env_alpha) * state->env_prev[bandId];
				state->env_prev[bandId] = env;

				const float comp = std::pow(env + 1e-9f, config.compression_gamma);

				const float hp_y = state->mod_hp_a0 * env + state->mod_hp_b1 * state->mod_hp_z1[bandId];
				state->mod_hp_z1[bandId] = env - state->mod_hp_c1 * hp_y;

				const float lp_y = state->mod_lp_a0 * hp_y + state->mod_lp_b1 * state->mod_lp_z1[bandId];
				state->mod_lp_z1[bandId] = hp_y - state->mod_lp_c1 * lp_y;

				outputs.cochlear_frame.envelope[bandId] = comp;
				outputs.cochlear_frame.modulation_power[bandId] = lp_y * lp_y;
				outputs.cochlear_frame.fine_phase[bandId] = config.output_phase ? state->phase[band.center_bin] : 0.0f;
			}

			outputs.cochlear_frame.timestamp = inputs.mono.timestamp;
		}

		void stop() {}
	};

} // namespace robotick
