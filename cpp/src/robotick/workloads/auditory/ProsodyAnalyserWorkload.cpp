// Copyright Robotick
// SPDX-License-Identifier: Apache-2.0

#include "robotick/api.h"
#include "robotick/systems/audio/AudioFrame.h"
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
	// ProsodyAnalyserWorkload
	// - Fixed-size FFT (N=512) via kissfftr
	// - Heap-free, real-time safe
	// - Fills ProsodyState including partials & HNR(dB)
	// - Pitch via CMNDf (YIN) + One-Euro smoothing
	// ======================================================

	struct ProsodyAnalyserConfig
	{
		// === Frame / FFT ===
		int fft_size = 512; // power-of-two; 256..1024 practical here
		bool use_hann_window = true;

		// === Pitch search (CMNDf/YIN) ===
		float min_f0_hz = 60.0f;
		float max_f0_hz = 2500.0f;
		float yin_threshold = 0.12f;   // absolute threshold (classic YIN)
		float pitch_conf_gate = 0.45f; // if confidence < gate → hold last (0..1)

		// === One-Euro smoothing for pitch ===
		// cutoff = min_cutoff + beta * |dx_hat|
		float one_euro_min_cutoff_hz = 4.0f; // base smoothing cutoff (Hz)
		float one_euro_beta = 0.1f;			 // speed coefficient
		float one_euro_dcutoff_hz = 4.0f;	 // derivative LPF cutoff (Hz)

		// === VAD / gate ===
		float vad_rms_threshold = 0.006f; // scale to your input (post-AGC)

		// === Partials ===
		int peak_search_half_width_bins = 1; // ±bins around integer multiple
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
		// Provide a mono frame each tick (0..N samples; analyser consumes available)
		AudioFrame mono;
	};

	struct ProsodyAnalyserOutputs
	{
		ProsodyState prosody_state;
	};

	struct ProsodyAnalyserState
	{
		// runtime
		int sample_rate = 44100;

		// FFT setup
		static constexpr int MaxN = 512; // keep in sync with default
		int N = 512;					 // actual size chosen from config
		int K = (MaxN / 2) + 1;			 // real FFT output length (for MaxN)
		kiss_fftr_cfg fft_cfg = nullptr;

		// Buffers (MaxN for fixed footprint)
		float time_in[MaxN] = {0.0f};
		float window[MaxN] = {0.0f};
		kiss_fft_cpx freq_out[(MaxN / 2) + 1] = {}; // K bins

		// YIN/CMNDf buffers (heap-free)
		float diff[MaxN + 1] = {0.0f};
		float cmndf[MaxN + 1] = {1.0f};

		// Rolling helpers
		float last_sample = 0.0f;			// for pre-emphasis
		float speaking_rate_tracker = 0.0f; // crude envelope tracker

		// One-Euro filter state for pitch
		bool pitch_initialized = false;
		float f0_raw_prev = 0.0f;
		float f0_smooth = 0.0f;
		float dx_smooth = 0.0f;

		// Utility
		inline static float clamp01(float v) { return v < 0.f ? 0.f : (v > 1.f ? 1.f : v); }
	};

	struct ProsodyAnalyserWorkload
	{
		ProsodyAnalyserConfig config;
		ProsodyAnalyserInputs inputs;
		ProsodyAnalyserOutputs outputs;
		State<ProsodyAnalyserState> state;

		// ---------- Helpers ----------
		static inline float safe_div(float num, float den, float def = 0.0f) { return (std::fabs(den) > 1e-20f) ? (num / den) : def; }

		static inline float alpha_from_cutoff(float cutoff_hz, float dt)
		{
			// One-Euro LPF alpha
			const float tau = 1.0f / (2.0f * 3.14159265358979323846f * std::max(1e-6f, cutoff_hz));
			return 1.0f / (1.0f + tau / std::max(1e-6f, dt));
		}

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

			// One-Euro init
			state->pitch_initialized = false;
			state->f0_raw_prev = 0.0f;
			state->f0_smooth = 0.0f;
			state->dx_smooth = 0.0f;
		}

		void start(float /*tick_rate_hz*/) { state->sample_rate = AudioSystem::get_sample_rate(); }

		// ---------- Pitch via CMNDf (YIN), heap-free using state buffers ----------
		// Returns f0 in Hz and fills out_conf in [0..1] (1 ~ strong periodicity).
		float estimate_pitch_hz_cmndf(const float* x, int n, int fs, float* out_conf)
		{
			if (out_conf)
				*out_conf = 0.0f;
			if (!x || n < 32 || fs <= 0)
				return 0.0f;

			const float maxF = std::max(config.max_f0_hz, 1.0f) * 1.10f; // small headroom
			const float minF = std::max(config.min_f0_hz, 1.0f);

			int min_lag = std::max(2, int((float)fs / maxF)); // high f → small lag
			int max_lag = std::max(3, int((float)fs / minF)); // low  f → big   lag
			int max_tau = std::min(max_lag, n - 3);
			if (min_lag >= max_tau)
				return 0.0f;

			// Local scratch to avoid touching state in a const context
			float diff[ProsodyAnalyserState::MaxN + 1];
			float cmndf[ProsodyAnalyserState::MaxN + 1];

			// Difference function d(tau)
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

			// CMNDf(tau)
			double running_sum = 0.0;
			cmndf[0] = 1.0f;
			for (int tau = 1; tau <= max_tau; ++tau)
			{
				running_sum += diff[tau];
				const double denom = (running_sum / (double)tau) + 1e-20;
				cmndf[tau] = (float)(diff[tau] / denom);
			}

			// Threshold and local-min walk (YIN)
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

			// Fallback: global minimum in search band
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

			// Parabolic interpolation around tau_est
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

			// Confidence: 1 - CMNDf at chosen tau
			const int t_idx = std::clamp((int)std::round(tau_refined), 1, max_tau);
			const float cm = std::clamp(cmndf[t_idx], 0.0f, 1.0f);
			if (out_conf)
				*out_conf = std::clamp(1.0f - cm, 0.0f, 1.0f);

			// Convert to frequency and sanity-check with loose bounds
			const float f0 = (float)fs / tau_refined;
			if (f0 < minF * 0.8f || f0 > maxF * 1.25f)
				return 0.0f;

			return f0;
		}

		void tick(const TickInfo& info)
		{
			auto& ps = outputs.prosody_state;

			const int fs = state->sample_rate;
			const int N = state->N;
			const int K = state->K;

			const int inCount = (int)inputs.mono.samples.size();
			if (inCount <= 0)
			{
				// No new data; output nothing this tick.
				return;
			}

			// --- Copy & preprocess into frame buffer (last N samples) ---
			const float* src = inputs.mono.samples.data();
			const int take = (inCount >= N) ? N : inCount;
			if (take < N)
			{
				// shift existing left and append
				const int shift = N - take;
				if (shift > 0)
				{
					for (int i = 0; i < shift; ++i)
						state->time_in[i] = state->time_in[i + take];
				}
				// append new samples to end
				for (int i = 0; i < take; ++i)
					state->time_in[N - take + i] = src[inCount - take + i];
			}
			else
			{
				// copy last N directly
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

			// --- Basic time-domain features (RMS, ZCR) ---
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

			// --- VAD ---
			ps.voiced = (rms >= config.vad_rms_threshold);
			ps.voiced_confidence = ProsodyAnalyserState::clamp01((rms - config.vad_rms_threshold) * 10.0f);

			// --- Pitch (YIN CMNDf) ---
			float f0_conf = 0.0f;
			const float f0_raw = estimate_pitch_hz_cmndf(state->time_in, N, fs, &f0_conf);

			// --- One-Euro smoothing with hold-on-low-confidence ---
			const float dt = (float)std::max(1e-6f, info.delta_time);
			float f0_s = state->f0_smooth;

			if (!state->pitch_initialized || true)
			{
				state->pitch_initialized = true;
				state->f0_raw_prev = f0_raw;
				state->dx_smooth = 0.0f;
				state->f0_smooth = f0_raw;
				f0_s = f0_raw;
			}
			else
			{
				// Only update if confidence is decent; else hold last smooth
				if (f0_raw > 0.0f && f0_conf >= config.pitch_conf_gate)
				{
					// derivative estimate
					const float dx = (f0_raw - state->f0_raw_prev) / dt;

					// low-pass derivative
					const float alpha_d = alpha_from_cutoff(config.one_euro_dcutoff_hz, dt);
					state->dx_smooth = (1.0f - alpha_d) * state->dx_smooth + alpha_d * dx;

					// adaptive cutoff
					const float cutoff = config.one_euro_min_cutoff_hz + config.one_euro_beta * std::fabs(state->dx_smooth);
					const float alpha = alpha_from_cutoff(cutoff, dt);

					// smooth f0
					state->f0_smooth = (1.0f - alpha) * state->f0_smooth + alpha * f0_raw;
					state->f0_raw_prev = f0_raw;
					f0_s = state->f0_smooth;
				}
				// else: keep f0_s as previous smoothed value (hold)
			}

			// Write pitch out (smoothed), slope from smoothed series
			const float prev_pitch = ps.pitch_hz;
			ps.pitch_hz = std::max(0.0f, f0_s);

			if (ps.pitch_hz > 0.0f && prev_pitch > 0.0f)
				ps.pitch_slope_hz_per_s = (ps.pitch_hz - prev_pitch) / std::max(1e-6f, dt);
			else
				ps.pitch_slope_hz_per_s = 0.0f;

			// Keep a sense of confidence (reuse voiced_confidence sensibly)
			ps.voiced_confidence = std::max(ps.voiced_confidence, f0_conf);

			// --- Window for FFT ---
			float tmp[ProsodyAnalyserState::MaxN];
			for (int i = 0; i < N; ++i)
				tmp[i] = state->time_in[i] * state->window[i];

			// --- Real FFT ---
			kiss_fftr(state->fft_cfg, tmp, state->freq_out);

			// --- Spectral magnitudes & summary stats ---
			double sum_mag = 0.0;
			double sum_f_mag = 0.0;
			double sum_f2_mag = 0.0;
			double sum_log = 0.0;
			double sum_lin = 0.0;
			double total_e = 0.0;

			const float bin_hz = (float)fs / (float)N;

			// mag buffer (stack-fixed) for reuse
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

			// Centroid / bandwidth
			const float centroid = (sum_mag > 0.0) ? (float)(sum_f_mag / sum_mag) : 0.0f;
			float bandwidth = 0.0f;
			if (sum_mag > 0.0)
			{
				const double m = (double)centroid;
				const double var = (sum_f2_mag / sum_mag) - m * m;
				bandwidth = (float)((var > 0.0) ? std::sqrt(var) : 0.0);
			}

			// Flatness (geo mean / arith mean)
			const double arith = sum_lin / (double)K;
			const double geo = std::exp(sum_log / (double)K);
			const float flatness = (float)((arith > 1e-30) ? (geo / arith) : 0.0);

			// Energy ratio (spectral rms vs time rms)
			const float spectral_rms = (float)std::sqrt(total_e / (double)K);
			const float energy_ratio = safe_div(spectral_rms, rms, 0.0f);

			// 85% rolloff
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

			// Coarse spectral slope proxy
			float spectral_slope = 0.0f;
			{
				const float bw = bandwidth;
				if (centroid > 1.0f && bw > 0.0f)
					spectral_slope = -20.0f * std::log10(centroid / (bw + 1e-6f));
				else
					spectral_slope = 0.0f;
			}

			// --- Partials & HNR (requires pitch) ---
			// store partial gains **relative to the fundamental magnitude**
			int partial_count = 0;
			float partial_gain_rel[Prosody::MaxPartials] = {0.0f};
			float partial_freq[Prosody::MaxPartials] = {0.0f};
			double harm_e = 0.0;

			// Measure fundamental magnitude near f0 (small ± search)
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
					{
						best_v0 = mag[kk];
					}
				}
				m_f0 = best_v0; // absolute magnitude of fundamental peak
			}

			// Peak-pick partials and store **relative** gains = m_i / (m_f0 + eps)
			if (ps.pitch_hz > 0.0f && m_f0 > 0.0f)
			{
				const int maxP = std::clamp((int)config.max_num_partials, 0, Prosody::MaxPartials);
				const int halfW = std::max(0, config.peak_search_half_width_bins);
				const float eps = 1e-12f;

				// search integer multiples 2*f0 .. (2+maxP-1)*f0
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
						partial_gain_rel[idx] = best_v / (m_f0 + eps); // <<< relative to fundamental
					}
				}

				// HNR energy can still use absolute magnitudes (mag^2 at chosen bins)
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

			// Compute HNR with absolute energies
			const double noise_e = std::max(1e-12, total_e - harm_e);
			float hnr_db = 0.0f;
			if (harm_e > 0.0)
				hnr_db = (float)(10.0 * std::log10(harm_e / noise_e));
			if (hnr_db < config.hnr_floor_db)
				hnr_db = config.hnr_floor_db;
			ps.harmonicity_hnr_db = hnr_db;

			// Write back to ProsodyState
			ps.partial_count = partial_count;
			ps.partial_freq_valid = true;
			ps.partial_gain.set_size(Prosody::MaxPartials);
			ps.partial_freq_hz.set_size(Prosody::MaxPartials);
			for (int i = 0; i < Prosody::MaxPartials; ++i)
			{
				ps.partial_gain[i] = (i < partial_count) ? partial_gain_rel[i] : 0.0f; // relative
				ps.partial_freq_hz[i] = (i < partial_count) ? partial_freq[i] : 0.0f;  // absolute Hz
			}

			// --- Speaking rate (very coarse envelope proxy) ---
			{
				const float alpha = ProsodyAnalyserState::clamp01(config.speaking_rate_decay);
				state->speaking_rate_tracker = alpha * state->speaking_rate_tracker + (1.0f - alpha) * spectral_rms;
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
		}
	};

} // namespace robotick
