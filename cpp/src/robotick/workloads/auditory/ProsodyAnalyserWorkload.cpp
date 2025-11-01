// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "robotick/api.h"
#include "robotick/systems/audio/AudioBuffer.h"
#include "robotick/systems/audio/AudioSystem.h"
#include "robotick/systems/auditory/ProsodyState.h"

#include <cfloat>
#include <cmath>
#include <kissfft/kiss_fftr.h>
#include <vector>

namespace robotick
{
	struct ProsodyAnalyserConfig
	{
		float min_f0_hz = 60.0f;
		float max_f0_hz = 400.0f;
		float vad_rms_threshold = 0.01f;
		float pre_emphasis = 0.0f;
		bool use_hann_window = true;
	};

	struct ProsodyAnalyserInputs
	{
		AudioBuffer512 mono;
	};

	struct ProsodyAnalyserOutputs
	{
		ProsodyState prosody_state;
	};

	struct ProsodyAnalyserState
	{
		// Sliding window parameters
		static constexpr size_t analysis_window_size = 2048;

		std::vector<float> sliding_buffer;
		size_t cursor = 0;
		bool filled = false;

		// Pitch tracking
		float prev_pitch_hz = 0.0f;
		bool prev_had_pitch = false;

		// Hann cache
		std::vector<float> hann;

		// FFT plan + buffers
		int fft_n = 0;
		kiss_fftr_cfg fft_cfg = nullptr;
		std::vector<float> fft_in;
		std::vector<kiss_fft_cpx> fft_out;

		bool ensure_fft(int n)
		{
			if (n < 16 || (n & 1))
				return false;
			if (fft_n == n && fft_cfg)
				return true;

			if (fft_cfg)
			{
				kiss_fftr_free(fft_cfg);
				fft_cfg = nullptr;
			}

			fft_cfg = kiss_fftr_alloc(n, 0, nullptr, nullptr);
			if (!fft_cfg)
				return false;

			fft_n = n;
			fft_in.resize(n);
			fft_out.resize(n / 2 + 1);
			return true;
		}
	};

	struct ProsodyAnalyserWorkload
	{
		ProsodyAnalyserConfig config;
		ProsodyAnalyserInputs inputs;
		ProsodyAnalyserOutputs outputs;

		State<ProsodyAnalyserState> state;

		static inline float sgnf(float x) { return (x > 0.f) - (x < 0.f); }

		void load()
		{
			AudioSystem::init();
			state->sliding_buffer.resize(ProsodyAnalyserState::analysis_window_size, 0.0f);
			state->cursor = 0;
			state->filled = false;
		}

		void ensure_hann(size_t n)
		{
			if (!config.use_hann_window)
				return;
			if (state->hann.size() == n)
				return;

			state->hann.resize(n);
			const float two_pi = 6.2831853f;
			for (size_t i = 0; i < n; ++i)
				state->hann[i] = 0.5f * (1.0f - std::cos(two_pi * (float)i / (float)(n - 1)));
		}

		float estimate_pitch_hz(const float* x, int n, int fs)
		{
			if (!x || n < 32)
				return 0.0f;

			const float maxF = std::max(config.max_f0_hz, 1.0f) * 1.10f;
			const float minF = std::max(config.min_f0_hz, 1.0f);

			int min_lag = std::max(2, int(fs / maxF));
			int max_lag = std::max(3, int(fs / minF));
			int max_tau = std::min({max_lag, n - 3});

			if (min_lag >= max_tau)
				return 0.0f;

			const float kThresh = 0.12f;

			static std::vector<float> diff, cmndf;
			diff.assign(max_tau + 1, 0.0f);
			cmndf.assign(max_tau + 1, 1.0f);

			for (int tau = 1; tau <= max_tau; ++tau)
			{
				double acc = 0.0;
				int limit = n - tau;
				for (int i = 0; i < limit; ++i)
				{
					const float d = x[i] - x[i + tau];
					acc += d * d;
				}
				diff[tau] = float(acc / std::max(1, limit));
			}
			diff[0] = 0.0f;

			double running_sum = 0.0;
			for (int tau = 1; tau <= max_tau; ++tau)
			{
				running_sum += diff[tau];
				cmndf[tau] = diff[tau] / (running_sum / tau + 1e-20f);
			}

			int tau_est = 0;
			for (int tau = min_lag; tau <= max_tau; ++tau)
			{
				if (cmndf[tau] < kThresh)
				{
					int t = tau;
					while (t + 1 <= max_tau && cmndf[t + 1] <= cmndf[t])
						++t;
					tau_est = t;
					break;
				}
			}

			if (tau_est == 0)
			{
				float best = FLT_MAX;
				for (int tau = min_lag; tau <= max_tau; ++tau)
					if (cmndf[tau] < best)
					{
						best = cmndf[tau];
						tau_est = tau;
					}
			}

			float tau_refined = float(tau_est);
			if (tau_est > 1 && tau_est < max_tau)
			{
				float ym1 = cmndf[tau_est - 1];
				float y0 = cmndf[tau_est];
				float yp1 = cmndf[tau_est + 1];
				float denom = (ym1 - 2 * y0 + yp1);
				if (std::fabs(denom) > 1e-12f)
				{
					float delta = 0.5f * (ym1 - yp1) / denom;
					tau_refined = float(tau_est) + std::clamp(delta, -1.0f, 1.0f);
				}
			}

			if (tau_refined <= 0.0f)
				return 0.0f;

			float f0 = fs / tau_refined;
			if (f0 < minF * 0.8f || f0 > maxF * 1.25f)
				return 0.0f;

			return f0;
		}

		void tick(const TickInfo& info)
		{
			const int fs = AudioSystem::get_sample_rate();
			const size_t N = inputs.mono.size();
			const float* x = inputs.mono.data();

			// Fill sliding buffer
			for (size_t i = 0; i < N; ++i)
			{
				state->sliding_buffer[state->cursor] = x[i];
				state->cursor = (state->cursor + 1) % ProsodyAnalyserState::analysis_window_size;
			}
			if (!state->filled && state->cursor == 0)
				state->filled = true;

			// Not enough data yet
			if (!state->filled)
			{
				outputs = ProsodyAnalyserOutputs{};
				return;
			}

			// Copy buffer to analysis frame
			std::vector<float> frame(ProsodyAnalyserState::analysis_window_size);
			size_t start = state->cursor;
			for (size_t i = 0; i < frame.size(); ++i)
				frame[i] = state->sliding_buffer[(start + i) % frame.size()];

			// Optional Hann
			if (config.use_hann_window)
			{
				ensure_hann(frame.size());
				for (size_t i = 0; i < frame.size(); ++i)
					frame[i] *= state->hann[i];
			}

			// --- RMS ---
			{
				double sumsq = 0.0;
				for (float v : frame)
					sumsq += v * v;
				outputs.prosody_state.rms = std::sqrt(sumsq / frame.size());
			}

			// --- ZCR ---
			{
				int crossings = 0;
				for (size_t i = 1; i < frame.size(); ++i)
					crossings += (sgnf(frame[i]) != sgnf(frame[i - 1]));
				outputs.prosody_state.zcr = float(crossings) / (frame.size() - 1);
			}

			// --- VAD ---
			outputs.prosody_state.voiced = (outputs.prosody_state.rms >= config.vad_rms_threshold);

			// --- Pitch ---
			{
				const float flat = outputs.prosody_state.spectral_flatness;
				const float zcr = outputs.prosody_state.zcr;
				const float ratio = outputs.prosody_state.spectral_energy_ratio;

				const bool tonality_good = (flat < 0.6f && zcr < 0.2f && ratio > 0.5f);
				const bool allow_pitch = outputs.prosody_state.voiced && tonality_good;

				const float f0 = allow_pitch ? estimate_pitch_hz(frame.data(), int(frame.size()), fs) : 0.0f;

				if (state->prev_had_pitch && f0 > 0.0f)
				{
					float dp = f0 - state->prev_pitch_hz;
					outputs.prosody_state.pitch_slope_hz_per_s = dp / std::max(1e-6f, info.delta_time);
				}
				else
				{
					outputs.prosody_state.pitch_slope_hz_per_s = 0.0f;
				}
				outputs.prosody_state.pitch_hz = f0;
				state->prev_pitch_hz = f0;
				state->prev_had_pitch = (f0 > 0.0f);
			}

			// --- Spectral features ---
			{
				const int evenN = int(frame.size()) & ~1;
				if (evenN >= 16 && state->ensure_fft(evenN))
				{
					for (int i = 0; i < evenN; ++i)
						state->fft_in[i] = frame[i];

					kiss_fftr(state->fft_cfg, state->fft_in.data(), state->fft_out.data());

					const int K = evenN / 2 + 1;
					const float bin_hz = float(fs) / evenN;

					double sum_mag = 0.0, sum_f_mag = 0.0;
					for (int k = 0; k < K; ++k)
					{
						const float re = state->fft_out[k].r;
						const float im = state->fft_out[k].i;
						const double mag = std::sqrt(re * re + im * im) + 1e-12;
						sum_mag += mag;
						sum_f_mag += mag * k * bin_hz;
					}

					if (sum_mag > 0.0)
					{
						const double centroid = sum_f_mag / sum_mag;
						outputs.prosody_state.spectral_centroid_hz = float(centroid);

						double sum_bw = 0.0, sum_log = 0.0, energy_sum = 0.0;
						for (int k = 0; k < K; ++k)
						{
							const float re = state->fft_out[k].r;
							const float im = state->fft_out[k].i;
							const double mag = std::sqrt(re * re + im * im) + 1e-12;
							const double fk = k * bin_hz;
							const double d = fk - centroid;
							sum_bw += d * d * mag;
							sum_log += std::log(mag);
							energy_sum += re * re + im * im;
						}

						const double gm = std::exp(sum_log / K);
						const double am = sum_mag / K;

						outputs.prosody_state.spectral_bandwidth_hz = float(std::sqrt(sum_bw / sum_mag));
						outputs.prosody_state.spectral_flatness = float(gm / (am + 1e-12));
						const float spectral_rms = float(std::sqrt(energy_sum / K));
						outputs.prosody_state.spectral_energy_rms = spectral_rms;
						outputs.prosody_state.spectral_energy_ratio = spectral_rms / (outputs.prosody_state.rms + 1e-6f);
					}
				}
			}
		}
	};

} // namespace robotick
