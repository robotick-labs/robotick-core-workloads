// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#include "robotick/api.h"
#include "robotick/systems/audio/AudioBuffer.h"
#include "robotick/systems/audio/AudioSystem.h"
#include "robotick/systems/auditory/ProsodyState.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <vector>

#include <kissfft/kiss_fftr.h>

namespace robotick
{
	// ==============================
	// Prosody (Prosody) Workload
	// ==============================

	struct ProsodyAnalyserConfig
	{
		// Pitch search range (human speech)
		float min_f0_hz = 60.0f;
		float max_f0_hz = 1000.0f;

		// Simple VAD threshold (RMS)
		float vad_rms_threshold = 0.01f;

		// Optional pre-emphasis factor for spectral features (not used yet)
		float pre_emphasis = 0.0f; // 0..0.97 typical

		// Whether to apply Hann window before spectral ops
		bool use_hann_window = true;
	};

	struct ProsodyAnalyserInputs
	{
		// Single-channel frame for analysis (e.g., 256/512 samples @ 44.1k)
		AudioBuffer512 mono;
	};

	struct ProsodyAnalyserOutputs
	{
		ProsodyState prosody_state;
	};

	struct ProsodyAnalyserState
	{
		float prev_pitch_hz = 0.0f;
		bool prev_had_pitch = false;

		// Cached Hann window to avoid recomputing
		std::vector<float> hann;

		// Reused FFT plan + buffers (lazy init per N)
		int fft_n = 0;
		kiss_fftr_cfg fft_cfg = nullptr;
		std::vector<float> fft_in;
		std::vector<kiss_fft_cpx> fft_out; // N/2+1 complex bins

		// Returns true when a valid plan is available for even n >= 16
		bool ensure_fft(int n)
		{
			// KissFFT real-FFT requires even n and a sensible minimum size
			if (n < 16 || (n & 1))
			{
				if (fft_cfg)
				{
					kiss_fftr_free(fft_cfg);
					fft_cfg = nullptr;
				}
				fft_n = 0;
				fft_in.clear();
				fft_out.clear();
				return false;
			}

			if (n == fft_n && fft_cfg)
				return true;

			if (fft_cfg)
			{
				kiss_fftr_free(fft_cfg);
				fft_cfg = nullptr;
			}

			fft_cfg = kiss_fftr_alloc(n, 0, nullptr, nullptr);
			if (!fft_cfg)
			{
				fft_n = 0;
				fft_in.clear();
				fft_out.clear();
				return false; // allocation failed (soft fail)
			}

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

		// -------------------------
		// Helpers
		// -------------------------
		static inline float sgnf(float x) { return (x > 0.f) - (x < 0.f); }

		void ensure_hann(size_t n)
		{
			if (!config.use_hann_window)
				return;
			if (state->hann.size() == n)
				return;
			state->hann.resize(n);
			const float two_pi = 6.28318530718f;
			for (size_t i = 0; i < n; ++i)
				state->hann[i] = 0.5f * (1.0f - std::cos(two_pi * (float)i / (float)(n - 1)));
		}

		// YIN with overlap-normalized difference and safe τ ceiling.
		// Uses unwindowed samples; call with the raw mono frame.
		float estimate_pitch_hz(const float* x, int n, int sample_rate)
		{
			if (!x || n < 32)
				return 0.0f;

			// --- Bounds from config, with headroom and half-frame cap ---
			const float maxF = std::max(config.max_f0_hz, 1.0f) * 1.10f; // 10% slack so 440 isn't clipped by 400
			const float minF = std::max(config.min_f0_hz, 1.0f);

			int min_lag = std::max(2, (int)std::floor((float)sample_rate / maxF)); // smallest τ (highest f)
			int max_lag = std::max(3, (int)std::floor((float)sample_rate / minF)); // largest τ (lowest f)

			// Critical: don't search beyond N/2 to avoid false minima at frame length.
			const int max_tau = std::min({max_lag, n / 2, n - 3});
			if (min_lag >= max_tau)
				return 0.0f;

			// Threshold for the first dip; 0.10–0.20 typical.
			const float kThresh = 0.12f;

			// Work buffers
			static std::vector<float> diff;	 // overlap-normalized difference
			static std::vector<float> cmndf; // cumulative mean normalized difference
			diff.assign(max_tau + 1, 0.0f);
			cmndf.assign(max_tau + 1, 1.0f);

			// --- Overlap-normalized difference: d'(τ) = (1/(N-τ)) * Σ (x[i] - x[i+τ])^2
			for (int tau = 1; tau <= max_tau; ++tau)
			{
				const int limit = n - tau;
				double acc = 0.0;
				for (int i = 0; i < limit; ++i)
				{
					const float d = x[i] - x[i + tau];
					acc += (double)d * (double)d;
				}
				diff[tau] = (float)(acc / (double)std::max(1, limit));
			}
			diff[0] = 0.0f;

			// --- CMNDF
			double running_sum = 0.0;
			cmndf[0] = 1.0f;
			for (int tau = 1; tau <= max_tau; ++tau)
			{
				running_sum += (double)diff[tau];
				const double denom = running_sum / (double)tau + 1e-20;
				cmndf[tau] = (float)((double)diff[tau] / denom);
			}

			// --- First dip below threshold (with local-min walk)
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

			// Fallback: global minimum in [min_lag, max_tau]
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

			// --- Parabolic refinement around τ (on CMNDF)
			float tau_refined = (float)tau_est;
			if (tau_est > 1 && tau_est < max_tau)
			{
				const float ym1 = cmndf[tau_est - 1];
				const float y0 = cmndf[tau_est + 0];
				const float yp1 = cmndf[tau_est + 1];
				const float denom = (ym1 - 2.0f * y0 + yp1);
				if (std::fabs(denom) > 1e-12f)
				{
					const float delta = 0.5f * (ym1 - yp1) / denom; // ∈ [-1,1]
					tau_refined = (float)tau_est + std::clamp(delta, -1.0f, 1.0f);
				}
			}

			if (tau_refined <= 0.0f)
				return 0.0f;

			const float f0 = (float)sample_rate / tau_refined;

			// Enforce final bounds with slack
			if (f0 < minF * 0.8f || f0 > maxF * 1.25f)
				return 0.0f;

			return f0;
		}

		// -------------------------
		// Lifecycle
		// -------------------------
		void load() { AudioSystem::init(); }

		void tick(const TickInfo& info)
		{
			const int fs = AudioSystem::get_sample_rate();
			const size_t N = inputs.mono.size();
			const float* x = inputs.mono.data();

			// Guard
			if (N == 0 || x == nullptr)
			{
				outputs = ProsodyAnalyserOutputs{}; // reset to zeros/stubs
				return;
			}

			// Optional windowing copy (also ensures contiguous temp buffer)
			std::vector<float> frame;
			frame.resize(N);
			if (config.use_hann_window)
			{
				ensure_hann(N);
				for (size_t i = 0; i < N; ++i)
					frame[i] = x[i] * state->hann[i];
			}
			else
			{
				// Copy (avoid touching inputs directly in case of aliasing)
				for (size_t i = 0; i < N; ++i)
					frame[i] = x[i];
			}

			// --- RMS ---
			{
				double sumsq = 0.0;
				for (size_t i = 0; i < N; ++i)
					sumsq += (double)frame[i] * (double)frame[i];
				outputs.prosody_state.rms = (float)std::sqrt(sumsq / (double)N);
			}

			// --- ZCR ---
			{
				int crossings = 0;
				for (size_t i = 1; i < N; ++i)
					crossings += (sgnf(frame[i]) != sgnf(frame[i - 1]));
				outputs.prosody_state.zcr = (float)crossings / (float)(N - 1);
			}

			// --- VAD (simple RMS threshold) ---
			outputs.prosody_state.voiced = (outputs.prosody_state.rms >= config.vad_rms_threshold);

			// --- Pitch (autocorrelation) + slope ---
			{
				const float f0 = outputs.prosody_state.voiced ? estimate_pitch_hz(x, (int)N, fs) : 0.0f;

				if (state->prev_had_pitch && f0 > 0.0f)
				{
					const float dp = f0 - state->prev_pitch_hz;
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

			// --- Spectral features (safe, even-N only) ---
			{
				// Prefer analyzing up to the input capacity (e.g., ring/window size),
				// but never exceed the current frame size.
				const int targetWin = (int)inputs.mono.capacity(); // e.g., 512
				int evenN = std::min((int)N, targetWin) & ~1;	   // force even length

				if (evenN >= 16 && state->ensure_fft(evenN) && state->fft_cfg)
				{
					// Fill FFT input with the first evenN samples of 'frame'
					for (int i = 0; i < evenN; ++i)
						state->fft_in[i] = frame[i];

					kiss_fftr(state->fft_cfg, state->fft_in.data(), state->fft_out.data());

					const int K = evenN / 2 + 1;
					const float bin_hz = (float)fs / (float)evenN;

					double sum_mag = 0.0;
					double sum_f_mag = 0.0;

					// First pass: centroid
					for (int k = 0; k < K; ++k)
					{
						const float re = state->fft_out[k].r;
						const float im = state->fft_out[k].i;
						const double mag = std::sqrt((double)re * re + (double)im * im) + 1e-12;
						sum_mag += mag;
						sum_f_mag += (double)k * bin_hz * mag;
					}

					if (sum_mag > 0.0)
					{
						const double centroid = sum_f_mag / sum_mag;
						outputs.prosody_state.spectral_centroid_hz = (float)centroid;

						// Second pass: bandwidth + flatness + energy
						double sum_bw = 0.0;
						double sum_log = 0.0;
						double energy_sum = 0.0;

						for (int k = 0; k < K; ++k)
						{
							const float re = state->fft_out[k].r;
							const float im = state->fft_out[k].i;
							const double mag = std::sqrt((double)re * re + (double)im * im) + 1e-12;
							const double fk = (double)k * bin_hz;
							const double d = fk - centroid;

							sum_bw += d * d * mag;
							sum_log += std::log(mag);
							energy_sum += re * re + im * im;
						}

						const double gm = std::exp(sum_log / (double)K);
						const double am = (sum_mag / (double)K);

						outputs.prosody_state.spectral_bandwidth_hz = (float)std::sqrt(sum_bw / sum_mag);
						outputs.prosody_state.spectral_flatness = (float)(gm / (am + 1e-12));

						// New: spectral energy and ratio
						const float spectral_rms = (float)std::sqrt(energy_sum / (double)K);
						outputs.prosody_state.spectral_energy_rms = spectral_rms;
						outputs.prosody_state.spectral_energy_ratio = spectral_rms / (outputs.prosody_state.rms + 1e-6f);
					}
					else
					{
						outputs.prosody_state.spectral_centroid_hz = 0.0f;
						outputs.prosody_state.spectral_bandwidth_hz = 0.0f;
						outputs.prosody_state.spectral_flatness = 0.0f;
						outputs.prosody_state.spectral_energy_rms = 0.0f;
						outputs.prosody_state.spectral_energy_ratio = 0.0f;
					}
				}
				else
				{
					// Preconditions not met (odd/too-small N) or plan alloc failed -> soft zero
					outputs.prosody_state.spectral_centroid_hz = 0.0f;
					outputs.prosody_state.spectral_bandwidth_hz = 0.0f;
					outputs.prosody_state.spectral_flatness = 0.0f;
				}
			}

			// (speaking_rate_sps, jitter, shimmer, hnr, formants) left as stubs for now
		}
	};

} // namespace robotick
