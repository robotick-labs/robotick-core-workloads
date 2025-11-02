// Copyright Robotick
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "robotick/api.h"
#include "robotick/systems/audio/AudioBuffer.h"
#include "robotick/systems/audio/AudioSystem.h"
#include "robotick/systems/auditory/ProsodyState.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <cstring>

#include <kissfft/kiss_fftr.h>

namespace robotick
{
	// ======================================================
	// ProsodyAnalyserWorkload (consistency-gated pitch, latched with grace)
	// - YIN/CMNDf pitch
	// - Stability window in cents (stddev + drift) with 1-outlier trimming
	// - Latching + off-grace to prevent crackly on/off
	// - Partials relative to fundamental; HNR from absolute peaks
	// - Heap-free, fixed footprint
	// ======================================================

	struct ProsodyAnalyserConfig
	{
		// === Frame / FFT ===
		int fft_size = 512; // power-of-two; up to 512 here
		bool use_hann_window = true;

		// === Pitch search (CMNDf/YIN) ===
		float min_f0_hz = 60.0f;
		float max_f0_hz = 2500.0f;
		float yin_threshold = 0.12f; // absolute threshold (classic YIN)

		// === Consistency Gate (no smoothing; require stability over a window) ===
		int cg_window_ms = 200;					 // lookback window size
		int cg_min_locked_ms = 50;				 // must be consistently voiced ≥ this long to lock
		int cg_off_grace_ms = 180;				 // keep output during brief instability (prevents crackle)
		float cg_min_confidence = 0.25f;		 // min CMNDf confidence to accept a sample
		float cg_max_spread_cents = 50.0f;		 // stddev in cents across window (after trimming)
		float cg_max_end_to_end_cents = 140.0f;	 // total drift allowed across window
		bool cg_gate_voiced_if_unstable = false; // avoid slamming voiced=false during brief breaks

		// === VAD / gate ===
		float vad_rms_threshold = 0.006f; // scale to input (post-AGC)

		// === Partials ===
		int peak_search_half_width_bins = 1; // ±bins round harmonic
		float partial_min_gain = 0.0f;		 // linear magnitude threshold
		uint8_t max_num_partials = Prosody::MaxPartials;

		// === HNR ===
		float hnr_floor_db = -60.0f; // clamp lower bound

		// === Speaking rate (coarse; placeholder) ===
		float speaking_rate_decay = 0.98f; // EWMA envelope proxy

		// === DC / pre-emphasis (optional) ===
		bool remove_dc = true;
		bool pre_emphasis = false;
		float pre_emph_coeff = 0.97f;
	};

	struct ProsodyAnalyserInputs
	{
		AudioBuffer512 mono; // provide a mono frame (0..N samples)
	};

	struct ProsodyAnalyserOutputs
	{
		ProsodyState prosody_state;
	};

	struct ProsodyAnalyserState
	{
		// runtime
		int sample_rate = 44100;

		// FFT setup (max footprint fixed)
		static constexpr int MaxN = 512; // keep in sync with default
		int N = 512;					 // actual size chosen from config
		int K = (MaxN / 2) + 1;			 // real FFT output length (for MaxN)
		kiss_fftr_cfg fft_cfg = nullptr;

		// Buffers (MaxN for fixed footprint)
		float time_in[MaxN] = {0.0f};
		float window[MaxN] = {0.0f};
		kiss_fft_cpx freq_out[(MaxN / 2) + 1] = {}; // K bins

		// Rolling helpers
		float last_sample = 0.0f;			// for pre-emphasis
		float speaking_rate_tracker = 0.0f; // crude envelope tracker

		// === Consistency-gate ring buffer (fixed-size) ===
		static constexpr int CG_MaxObs = 256;
		int cg_head = 0;				// next write index
		int cg_size = 0;				// number of valid samples
		float cg_f0[CG_MaxObs] = {0};	// Hz (0 if no estimate)
		float cg_conf[CG_MaxObs] = {0}; // 0..1 (CMNDf confidence)
		float cg_rms[CG_MaxObs] = {0};	// block RMS
		float cg_dt[CG_MaxObs] = {0};	// seconds

		// Latch state to avoid crackle
		bool cg_locked = false;
		float cg_hold_f0_hz = 0.0f;		 // robust f0 to hold during short instability
		float cg_unstable_time_s = 0.0f; // time spent unstable (for grace)

		// Utility
		inline static float clamp01(float v) { return v < 0.f ? 0.f : (v > 1.f ? 1.f : v); }
		inline static float log2f_safe(float x) { return (x > 1e-20f) ? (std::log(x) / std::log(2.0f)) : -1e9f; }
		inline static float hz_to_cents(float hz) { return 1200.0f * log2f_safe(std::max(hz, 1e-12f)); }
		inline static float cents_to_hz(float c) { return std::pow(2.0f, c / 1200.0f); }
	};

	struct ProsodyAnalyserWorkload
	{
		ProsodyAnalyserConfig config;
		ProsodyAnalyserInputs inputs;
		ProsodyAnalyserOutputs outputs;
		State<ProsodyAnalyserState> state;

		// ---------- Helpers ----------
		static inline float safe_div(float num, float den, float def = 0.0f) { return (std::fabs(den) > 1e-20f) ? (num / den) : def; }

		void build_window()
		{
			const int N = state->N;
			if (!config.use_hann_window)
			{
				for (int i = 0; i < N; ++i)
					state->window[i] = 1.0f;
				return;
			}
			const float two_pi = 6.28318530717958647692f;
			for (int n = 0; n < N; ++n)
				state->window[n] = 0.5f * (1.0f - std::cos(two_pi * (float)n / (float)(N - 1)));
		}

		void load()
		{
			AudioSystem::init();

			// Clamp/choose N to our Max
			int N = config.fft_size;
			if (N <= 0 || N > ProsodyAnalyserState::MaxN || (N & (N - 1)) != 0)
				N = ProsodyAnalyserState::MaxN;
			state->N = N;
			state->K = (N / 2) + 1;

			// FFT plan (created once)
			if (state->fft_cfg)
			{
				kiss_fftr_free(state->fft_cfg);
				state->fft_cfg = nullptr;
			}
			state->fft_cfg = kiss_fftr_alloc(N, 0, nullptr, nullptr);

			// Window
			build_window();

			state->last_sample = 0.0f;
			state->speaking_rate_tracker = 0.0f;

			// Clear outputs
			std::memset(&outputs.prosody_state, 0, sizeof(outputs.prosody_state));

			// Reset consistency-gate ring + latch
			state->cg_head = 0;
			state->cg_size = 0;
			state->cg_locked = false;
			state->cg_hold_f0_hz = 0.0f;
			state->cg_unstable_time_s = 0.0f;
			for (int i = 0; i < ProsodyAnalyserState::CG_MaxObs; ++i)
			{
				state->cg_f0[i] = 0.0f;
				state->cg_conf[i] = 0.0f;
				state->cg_rms[i] = 0.0f;
				state->cg_dt[i] = 0.0f;
			}
		}

		void start(float /*tick_rate_hz*/) { state->sample_rate = AudioSystem::get_sample_rate(); }

		// ---------- Pitch via CMNDf (YIN), heap-free using local stack ----------
		float estimate_pitch_hz_cmndf(const float* x, int n, int fs, float* out_conf)
		{
			if (out_conf)
				*out_conf = 0.0f;
			if (!x || n < 32 || fs <= 0)
				return 0.0f;

			const float maxF = std::max(config.max_f0_hz, 1.0f) * 1.10f;
			const float minF = std::max(config.min_f0_hz, 1.0f);

			int min_lag = std::max(2, int((float)fs / maxF));
			int max_lag = std::max(3, int((float)fs / minF));
			int max_tau = std::min(max_lag, n - 3);
			if (min_lag >= max_tau)
				return 0.0f;

			float diff[ProsodyAnalyserState::MaxN + 1];
			float cmndf[ProsodyAnalyserState::MaxN + 1];

			for (int tau = 1; tau <= max_tau; ++tau)
			{
				double acc = 0.0;
				const int limit = n - tau;
				int i = 0;
				for (; i + 3 < limit; i += 4)
				{
					const float d0 = x[i] - x[i + tau];
					const float d1 = x[i + 1] - x[i + tau + 1];
					const float d2 = x[i + 2] - x[i + tau + 2];
					const float d3 = x[i + 3] - x[i + tau + 3];
					acc += (double)d0 * d0 + (double)d1 * d1 + (double)d2 * d2 + (double)d3 * d3;
				}
				for (; i < limit; ++i)
				{
					const float d = x[i] - x[i + tau];
					acc += (double)d * (double)d;
				}
				diff[tau] = (float)(acc / std::max(1, limit));
			}
			diff[0] = 0.0f;

			double running_sum = 0.0;
			cmndf[0] = 1.0f;
			for (int tau = 1; tau <= max_tau; ++tau)
			{
				running_sum += diff[tau];
				const double denom = (running_sum / (double)tau) + 1e-20;
				cmndf[tau] = (float)(diff[tau] / denom);
			}

			const float kThresh = config.yin_threshold;
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

			float tau_refined = (float)tau_est;
			if (tau_est > 1 && tau_est < max_tau)
			{
				const float ym1 = cmndf[tau_est - 1];
				const float y0 = cmndf[tau_est];
				const float yp1 = cmndf[tau_est + 1];
				const float denom = (ym1 - 2.0f * y0 + yp1);
				if (std::fabs(denom) > 1e-12f)
				{
					const float delta = 0.5f * (ym1 - yp1) / denom;
					tau_refined = (float)tau_est + std::clamp(delta, -1.0f, 1.0f);
				}
			}
			if (!(tau_refined > 0.0f))
				return 0.0f;

			const int t_idx = std::clamp((int)std::round(tau_refined), 1, max_tau);
			const float cm = std::clamp(cmndf[t_idx], 0.0f, 1.0f);
			if (out_conf)
				*out_conf = std::clamp(1.0f - cm, 0.0f, 1.0f);

			const float f0 = (float)fs / tau_refined;
			if (f0 < config.min_f0_hz * 0.8f || f0 > config.max_f0_hz * 1.25f)
				return 0.0f;

			return f0;
		}

		void tick(const TickInfo& info)
		{
			auto& ps = outputs.prosody_state;

			const int fs = state->sample_rate;
			const int N = state->N;
			const int K = state->K;

			const int inCount = (int)inputs.mono.size();
			if (inCount <= 0)
				return;

			// --- Copy & preprocess last N samples ---
			const float* src = inputs.mono.data();
			const int take = (inCount >= N) ? N : inCount;
			if (take < N)
			{
				const int shift = N - take;
				if (shift > 0)
				{
					for (int i = 0; i < shift; ++i)
						state->time_in[i] = state->time_in[i + take];
				}
				for (int i = 0; i < take; ++i)
					state->time_in[N - take + i] = src[inCount - take + i];
			}
			else
			{
				for (int i = 0; i < N; ++i)
					state->time_in[i] = src[inCount - N + i];
			}

			// Optional pre-emphasis
			if (config.remove_dc || config.pre_emphasis)
			{
				float prev = state->last_sample;
				for (int i = 0; i < N; ++i)
				{
					const float x = state->time_in[i];
					if (config.pre_emphasis)
					{
						const float y = x - config.pre_emph_coeff * prev;
						state->time_in[i] = y;
						prev = x;
					}
				}
				state->last_sample = state->time_in[N - 1];
			}

			// Remove per-frame DC by subtracting mean
			if (config.remove_dc)
			{
				double sum = 0.0;
				for (int i = 0; i < N; ++i)
					sum += state->time_in[i];
				const float mean = (float)(sum / (double)N);
				for (int i = 0; i < N; ++i)
					state->time_in[i] -= mean;
			}

			// --- Time-domain features (RMS, ZCR) ---
			double e_sum = 0.0;
			int zc = 0;
			{
				float prev = state->time_in[0];
				for (int i = 0; i < N; ++i)
				{
					const float x = state->time_in[i];
					e_sum += (double)x * (double)x;
					if ((x >= 0.0f && prev < 0.0f) || (x < 0.0f && prev >= 0.0f))
						++zc;
					prev = x;
				}
			}
			const float rms = (float)std::sqrt(e_sum / (double)N);
			ps.rms = rms;
			ps.zcr = (float)zc / (float)N;

			// --- VAD (time-domain gate) ---
			const bool voiced_td = (rms >= config.vad_rms_threshold);

			// --- Pitch (YIN CMNDf) ---
			float f0_conf = 0.0f;
			const float f0_raw = estimate_pitch_hz_cmndf(state->time_in, N, fs, &f0_conf);

			// --- Push observation into ring ---
			const float dt = (float)std::max(1e-6f, info.delta_time);
			{
				const int i = state->cg_head;
				state->cg_f0[i] = f0_raw;
				state->cg_conf[i] = f0_conf;
				state->cg_rms[i] = rms;
				state->cg_dt[i] = dt;
				state->cg_head = (state->cg_head + 1) % ProsodyAnalyserState::CG_MaxObs;
				state->cg_size = std::min(state->cg_size + 1, ProsodyAnalyserState::CG_MaxObs);
			}

			// --- Collect window, compute trimmed stats in cents ---
			const float window_s = 0.001f * (float)std::max(0, config.cg_window_ms);
			const float min_lock_s = 0.001f * (float)std::max(0, config.cg_min_locked_ms);

			float total_s = 0.0f;
			float locked_s = 0.0f;
			float cents_buf[ProsodyAnalyserState::CG_MaxObs];
			int cents_n = 0;
			float first_cents = 0.0f, last_cents = 0.0f;
			bool first_set = false;

			int idx = (state->cg_head - 1 + ProsodyAnalyserState::CG_MaxObs) % ProsodyAnalyserState::CG_MaxObs;
			for (int n = 0; n < state->cg_size; ++n)
			{
				const float dti = state->cg_dt[idx];
				const float f0i = state->cg_f0[idx];
				const float cfi = state->cg_conf[idx];

				total_s += dti;

				if (f0i > 0.0f && cfi >= config.cg_min_confidence)
				{
					const float ci = ProsodyAnalyserState::hz_to_cents(f0i);
					cents_buf[cents_n++] = ci;
					locked_s += dti;
					if (!first_set)
					{
						first_cents = ci;
						last_cents = ci;
						first_set = true;
					}
					else
					{
						last_cents = ci;
					}
				}

				if (total_s >= window_s)
					break;
				idx = (idx - 1 + ProsodyAnalyserState::CG_MaxObs) % ProsodyAnalyserState::CG_MaxObs;
			}

			// Robust stats: trim the single largest deviation if we have enough samples
			auto compute_trimmed_std = [&](const float* arr, int n, double mean_c) -> double
			{
				if (n <= 2)
					return 1e9;
				int max_i = 0;
				double max_dev = 0.0;
				for (int i = 0; i < n; ++i)
				{
					const double d = std::fabs((double)arr[i] - mean_c);
					if (d > max_dev)
					{
						max_dev = d;
						max_i = i;
					}
				}
				double var = 0.0;
				int cnt = 0;
				for (int i = 0; i < n; ++i)
				{
					if (i == max_i && n >= 5)
						continue; // drop 1 outlier only if enough points
					const double d = (double)arr[i] - mean_c;
					var += d * d;
					++cnt;
				}
				return std::sqrt(var / std::max(1, cnt));
			};

			const float off_grace_s = 0.001f * (float)std::max(0, config.cg_off_grace_ms);
			bool stable = false;
			float stable_mean_hz = 0.0f;

			if (locked_s >= min_lock_s && cents_n >= 2)
			{
				double sumc = 0.0;
				for (int i = 0; i < cents_n; ++i)
					sumc += cents_buf[i];
				const double mean_c = sumc / (double)cents_n;

				const double std_c_trim = compute_trimmed_std(cents_buf, cents_n, mean_c);
				const float drift_c = std::fabs(last_cents - first_cents);

				stable = (std_c_trim <= (double)config.cg_max_spread_cents) && (drift_c <= config.cg_max_end_to_end_cents);

				if (stable)
				{
					stable_mean_hz = ProsodyAnalyserState::cents_to_hz((float)mean_c);
					// clamp to plausible bounds
					if (!(stable_mean_hz > 0.0f) || stable_mean_hz < config.min_f0_hz * 0.5f || stable_mean_hz > config.max_f0_hz * 2.0f)
					{
						stable = false;
					}
				}
			}

			// --- Latch state machine (prevents crackly off/on) ---
			float pitch_out_hz = 0.0f;
			bool voiced_out = voiced_td;

			if (stable)
			{
				state->cg_locked = true;
				state->cg_unstable_time_s = 0.0f;
				state->cg_hold_f0_hz = stable_mean_hz;
				pitch_out_hz = stable_mean_hz;
			}
			else
			{
				if (state->cg_locked)
				{
					state->cg_unstable_time_s += dt;
					if (state->cg_unstable_time_s <= off_grace_s && state->cg_hold_f0_hz > 0.0f)
					{
						// Within grace: keep last robust f0 (smooth experience)
						pitch_out_hz = state->cg_hold_f0_hz;
					}
					else
					{
						// Exceeded grace: unlock
						state->cg_locked = false;
						state->cg_hold_f0_hz = 0.0f;
						state->cg_unstable_time_s = 0.0f;
					}
				}

				if (!state->cg_locked && config.cg_gate_voiced_if_unstable)
					voiced_out = false;
			}

			// Final voiced/confidence
			ps.voiced = voiced_out;
			ps.voiced_confidence = ProsodyAnalyserState::clamp01(std::max(ps.voiced_confidence, f0_conf));

			// Publish pitch (held or 0)
			const float prev_pitch = ps.pitch_hz;
			ps.pitch_hz = std::max(0.0f, pitch_out_hz);
			ps.pitch_slope_hz_per_s = (ps.pitch_hz > 0.0f && prev_pitch > 0.0f) ? (ps.pitch_hz - prev_pitch) / std::max(1e-6f, dt) : 0.0f;

			// --- Window for FFT ---
			float tmp[ProsodyAnalyserState::MaxN];
			for (int i = 0; i < N; ++i)
				tmp[i] = state->time_in[i] * state->window[i];

			// --- Real FFT ---
			kiss_fftr(state->fft_cfg, tmp, state->freq_out);

			// --- Spectral magnitudes & summary stats ---
			double sum_mag = 0.0, sum_f_mag = 0.0, sum_f2_mag = 0.0;
			double sum_log = 0.0, sum_lin = 0.0, total_e = 0.0;

			const float bin_hz = (float)fs / (float)N;

			float mag[ProsodyAnalyserState::MaxN / 2 + 1];
			for (int k = 0; k < K; ++k)
			{
				const float re = state->freq_out[k].r;
				const float im = state->freq_out[k].i;
				const float m = std::sqrt(re * re + im * im);
				mag[k] = m;

				const double f = (double)k * (double)bin_hz;
				sum_mag += (double)m;
				sum_f_mag += f * (double)m;
				sum_f2_mag += (f * f) * (double)m;

				sum_lin += (double)m + 1e-20;
				sum_log += std::log((double)m + 1e-20);

				total_e += (double)m * (double)m;
			}

			const float centroid = (sum_mag > 0.0) ? (float)(sum_f_mag / sum_mag) : 0.0f;
			float bandwidth = 0.0f;
			if (sum_mag > 0.0)
			{
				const double m = (double)centroid;
				const double var = (sum_f2_mag / sum_mag) - m * m;
				bandwidth = (float)((var > 0.0) ? std::sqrt(var) : 0.0);
			}

			const double arith = sum_lin / (double)K;
			const double geo = std::exp(sum_log / (double)K);
			const float flatness = (float)((arith > 1e-30) ? (geo / arith) : 0.0);

			const float spectral_rms = (float)std::sqrt(total_e / (double)K);
			const float energy_ratio = safe_div(spectral_rms, rms, 0.0f);

			float rolloff_hz = 0.0f;
			if (total_e > 0.0)
			{
				const double thresh = 0.85 * total_e;
				double cum = 0.0;
				for (int k = 0; k < K; ++k)
				{
					cum += (double)mag[k] * (double)mag[k];
					if (cum >= thresh)
					{
						rolloff_hz = (float)k * bin_hz;
						break;
					}
				}
			}

			float spectral_slope = 0.0f;
			{
				const float bw = bandwidth;
				if (centroid > 1.0f && bw > 0.0f)
					spectral_slope = -20.0f * std::log10(centroid / (bw + 1e-6f));
				else
					spectral_slope = 0.0f;
			}

			// --- Partials & HNR (requires pitch) ---
			int partial_count = 0;
			float partial_gain_rel[Prosody::MaxPartials] = {0};
			float partial_freq[Prosody::MaxPartials] = {0};
			double harm_e = 0.0;

			float m_f0 = 0.0f;
			if (ps.pitch_hz > 0.0f)
			{
				const int k0_guess = (int)std::round(ps.pitch_hz / bin_hz);
				const int k0 = std::clamp(k0_guess, 1, K - 2);

				float best_v0 = mag[k0];
				for (int dk = -1; dk <= 1; ++dk)
				{
					const int kk = k0 + dk;
					if (kk > 0 && kk < K && mag[kk] > best_v0)
						best_v0 = mag[kk];
				}
				m_f0 = best_v0;
			}

			if (ps.pitch_hz > 0.0f && m_f0 > 0.0f)
			{
				const int maxP = std::clamp((int)config.max_num_partials, 0, Prosody::MaxPartials);
				const int halfW = std::max(0, config.peak_search_half_width_bins);
				const float eps = 1e-12f;

				for (int h = 2; h < 2 + maxP; ++h)
				{
					const float target_hz = ps.pitch_hz * (float)h;
					const int kc_guess = (int)std::round(target_hz / bin_hz);
					if (kc_guess <= 1 || kc_guess >= K - 2)
						break;

					int best_k = kc_guess;
					float best_v = mag[kc_guess];
					for (int dk = -halfW; dk <= halfW; ++dk)
					{
						const int kk = kc_guess + dk;
						if (kk > 0 && kk < K && mag[kk] > best_v)
						{
							best_v = mag[kk];
							best_k = kk;
						}
					}

					if (best_v > config.partial_min_gain)
					{
						const int idx = partial_count++;
						if (idx >= maxP)
							break;
						partial_freq[idx] = (float)best_k * bin_hz;	   // absolute Hz
						partial_gain_rel[idx] = best_v / (m_f0 + eps); // relative to fundamental
					}
				}

				for (int i = 0; i < partial_count; ++i)
				{
					const int k = (int)std::round(partial_freq[i] / bin_hz);
					if (k >= 0 && k < K)
					{
						const double e = (double)mag[k] * (double)mag[k];
						harm_e += e;
					}
				}
			}

			const double noise_e = std::max(1e-12, total_e - harm_e);
			float hnr_db = 0.0f;
			if (harm_e > 0.0)
				hnr_db = (float)(10.0 * std::log10(harm_e / noise_e));
			if (hnr_db < config.hnr_floor_db)
				hnr_db = config.hnr_floor_db;

			// --- Speaking rate (very coarse envelope proxy) ---
			{
				const float a = ProsodyAnalyserState::clamp01(config.speaking_rate_decay);
				state->speaking_rate_tracker = a * state->speaking_rate_tracker + (1.0f - a) * spectral_rms;
				ps.speaking_rate_sps = state->speaking_rate_tracker; // placeholder
			}

			// --- Write back to ProsodyState ---
			ps.spectral_energy_rms = spectral_rms;
			ps.spectral_energy_ratio = energy_ratio;
			ps.spectral_centroid_hz = centroid;
			ps.spectral_bandwidth_hz = bandwidth;
			ps.spectral_flatness = flatness;
			ps.spectral_rolloff_hz = rolloff_hz;
			ps.spectral_slope = spectral_slope;

			ps.harmonicity_hnr_db = hnr_db;

			ps.partial_count = partial_count;
			ps.partial_freq_valid = true;
			ps.partial_gain.set_size(Prosody::MaxPartials);
			ps.partial_freq_hz.set_size(Prosody::MaxPartials);
			for (int i = 0; i < Prosody::MaxPartials; ++i)
			{
				ps.partial_gain[i] = (i < partial_count) ? partial_gain_rel[i] : 0.0f;
				ps.partial_freq_hz[i] = (i < partial_count) ? partial_freq[i] : 0.0f;
			}
		}
	};

} // namespace robotick
