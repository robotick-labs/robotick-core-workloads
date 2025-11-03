// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0
//
// TemporalGroupingWorkload v0.2
//  - Adds temporal coherence, modulation-rate (2–10 Hz) via Goertzel,
//    and multi-source deconfliction (soft harmonic masking)
//  - MCU-friendly: fixed arrays, no exceptions, no STL in hot path
//
// Inputs:   TemporalGroupingInputs { CochlearFrame cochlear_frame; }
// Outputs:  TemporalGroupingOutputs { FixedVector<SourceCandidate, MaxSources> sources; }
// Tick:     tick(const TickInfo&)

#pragma once

#include "robotick/api.h"

#include "robotick/systems/audio/AudioSystem.h"
#include "robotick/systems/auditory/CochlearFrame.h"
#include "robotick/systems/auditory/SourceCandidate.h"

#include <cmath>

namespace robotick
{
	struct TemporalGroupingConfig
	{
		// Match CochlearTransform bands:
		float fmin_hz = 50.0f;
		float fmax_hz = 3500.0f;
		uint16_t num_bands = 128;

		// Fundamental search
		float f0_min_hz = 60.0f;
		float f0_max_hz = 1200.0f;

		// Harmonic sieve
		uint8_t max_harmonics = 10;
		float harmonic_tolerance_cents = 35.0f; // ± window for “near-harmonic”

		// Selection / gating
		uint8_t max_sources = 3;
		float min_harmonicity = 0.15f;
		float min_amplitude = 1e-4f;

		// Temporal smoothing
		float smooth_alpha = 0.5f;

		// History (for temporal coherence / modulation)
		uint8_t history_frames = 16;		  // ~200 ms at ~80 Hz
		float coherence_min_window_s = 0.08f; // guard if tick rate is very high

		// Deconfliction (masking)
		float reuse_penalty = 0.45f; // 0..1 additional penalty if bin already claimed

		// Modulation-rate probing (Goertzel)
		// Discrete targets keep cost tiny and deterministic.
		uint8_t modulation_bins = 7; // we’ll probe: {2,3,4,5,6,8,10} Hz
	};

	struct TemporalGroupingInputs
	{
		CochlearFrame cochlear_frame;
	};

	struct TemporalGroupingOutputs
	{
		SourceCandidates8 source_candidates;
	};

	// ---- Workload ----

	struct TemporalGroupingWorkload
	{
		TemporalGroupingConfig config;
		TemporalGroupingInputs inputs;
		TemporalGroupingOutputs outputs;

		// --- Internal state ---

		struct HistEntry
		{
			float envelope[256]; // up to MaxBands
			double timestamp = 0.0;
		};

		static constexpr uint16_t MaxBands = 256;
		static constexpr uint8_t MaxHistory = 32;

		struct State
		{
			HistEntry history[MaxHistory];
			uint8_t history_count = 0;
			uint8_t history_head = 0;
			bool initialised = false;

			// Soft “used energy” mask for deconfliction (per band)
			float claimed_energy[MaxBands];
			// Tracks for EMA stabilisation
			struct Track
			{
				bool active = false;
				uint8_t id = 0;
				float pitch_hz = 0.0f;
				float amplitude = 0.0f;
				float centroid_hz = 0.0f;
				float bandwidth_hz = 0.0f;
				float harmonicity = 0.0f;
				float temporal_coherence = 0.0f;
				float modulation_rate = 0.0f;
				double last_timestamp = 0.0;
			};
			static constexpr uint8_t MaxTracks = 8;
			Track tracks[MaxTracks];
			uint8_t next_track_id = 1;

			void reset_claims(uint16_t nb)
			{
				for (uint16_t i = 0; i < nb && i < MaxBands; ++i)
					claimed_energy[i] = 0.0f;
			}
		};

		State state;

		// ===== Utility =====

		static inline float clampf(float v, float lo, float hi)
		{
			if (v < lo)
				return lo;
			if (v > hi)
				return hi;
			return v;
		}

		inline float band_hz(uint16_t i) const
		{
			const float n = (float)(config.num_bands - 1);
			if (n <= 0.0f)
				return config.fmin_hz;
			const float r = std::pow(config.fmax_hz / config.fmin_hz, 1.0f / n);
			return config.fmin_hz * std::pow(r, (float)i);
		}

		inline uint16_t band_index_for_hz(float hz) const
		{
			if (hz <= config.fmin_hz)
				return 0;
			if (hz >= config.fmax_hz)
				return (uint16_t)(config.num_bands - 1);
			const float n = (float)(config.num_bands - 1);
			const float r = std::pow(config.fmax_hz / config.fmin_hz, 1.0f / n);
			const float i = std::log(hz / config.fmin_hz) / std::log(r);
			int idx = (int)(i + 0.5f);
			if (idx < 0)
				idx = 0;
			if (idx >= (int)config.num_bands)
				idx = (int)config.num_bands - 1;
			return (uint16_t)idx;
		}

		inline float cents_between(float f1, float f2) const
		{
			if (f1 <= 0.0f || f2 <= 0.0f)
				return 1200.0f;
			const float ratio = f2 / f1;
			return 1200.0f * (float)std::log2((double)ratio);
		}

		inline float frame_energy(const CochlearFrame& f) const
		{
			const uint16_t nb = (uint16_t)config.num_bands;
			float s = 0.0f;
			for (uint16_t i = 0; i < nb; ++i)
				s += f.envelope[i];
			return s;
		}

		// ---- History ring ----

		void push_history(const CochlearFrame& f)
		{
			const uint8_t cap = (config.history_frames > MaxHistory) ? MaxHistory : config.history_frames;
			if (cap == 0)
				return;

			if (!state.initialised)
			{
				std::memset(&state, 0, sizeof(state));
				state.initialised = true;
			}

			state.history_head = (uint8_t)((state.history_head + 1) % cap);
			HistEntry& e = state.history[state.history_head];
			const uint16_t nb = (uint16_t)config.num_bands;
			for (uint16_t i = 0; i < nb && i < MaxBands; ++i)
				e.envelope[i] = f.envelope[i];
			e.timestamp = f.timestamp;

			if (state.history_count < cap)
				++state.history_count;
		}

		// ---- Temporal coherence over grouped bands ----
		// Returns 0..1. We correlate each band’s recent envelope with the
		// band-mean envelope across the same set, then average the correlations.
		float temporal_coherence_score(const uint16_t* band_indices, uint8_t band_count, float tick_rate_hz, float& out_group_envelope) const
		{
			out_group_envelope = 0.0f;
			const uint8_t N = state.history_count;
			if (N < 3 || band_count == 0)
				return 0.0f;

			// Ensure we have enough time covered
			const double t0 = state.history[state.history_head].timestamp;
			const double tN = state.history[(state.history_head + 256 - (N - 1)) % 256].timestamp;
			if ((t0 - tN) < (double)config.coherence_min_window_s)
				return 0.0f;

			// Build per-frame group mean envelope
			float mean_env[MaxHistory];
			for (uint8_t k = 0; k < N; ++k)
			{
				const uint8_t idx = (uint8_t)((state.history_head + 256 - k) % 256);
				const HistEntry& he = state.history[idx];
				float s = 0.0f;
				for (uint8_t b = 0; b < band_count; ++b)
					s += he.envelope[band_indices[b]];
				mean_env[N - 1 - k] = s / (float)band_count;
				out_group_envelope += mean_env[N - 1 - k];
			}
			out_group_envelope /= (float)N;

			// Corr(each band, mean) averaged
			float corr_sum = 0.0f;
			for (uint8_t b = 0; b < band_count; ++b)
			{
				float x[MaxHistory];
				for (uint8_t k = 0; k < N; ++k)
				{
					const uint8_t idx = (uint8_t)((state.history_head + 256 - k) % 256);
					x[N - 1 - k] = state.history[idx].envelope[band_indices[b]];
				}
				// Pearson r between x and mean_env
				float mx = 0.0f, mm = 0.0f;
				for (uint8_t k = 0; k < N; ++k)
				{
					mx += x[k];
					mm += mean_env[k];
				}
				mx /= (float)N;
				mm /= (float)N;
				float num = 0.0f, denx = 0.0f, denm = 0.0f;
				for (uint8_t k = 0; k < N; ++k)
				{
					const float dx = x[k] - mx;
					const float dm = mean_env[k] - mm;
					num += dx * dm;
					denx += dx * dx;
					denm += dm * dm;
				}
				const float den = std::sqrt(denx * denm) + 1e-9f;
				const float r = (den > 0.0f) ? (num / den) : 0.0f;
				corr_sum += r * 0.5f + 0.5f; // map -1..1 → 0..1
			}

			return corr_sum / (float)band_count;
		}

		// ---- Modulation-rate via Goertzel over group envelope (2..10 Hz) ----
		float estimate_modulation_rate_hz(float tick_rate_hz, const uint16_t* band_indices, uint8_t band_count) const
		{
			const uint8_t N = state.history_count;
			if (N < 6 || band_count == 0)
				return 0.0f;

			// Build group envelope (mean across bands) as y[k]
			float y[MaxHistory];
			for (uint8_t k = 0; k < N; ++k)
			{
				const uint8_t idx = (uint8_t)((state.history_head + 256 - k) % 256);
				const HistEntry& he = state.history[idx];
				float s = 0.0f;
				for (uint8_t b = 0; b < band_count; ++b)
					s += he.envelope[band_indices[b]];
				y[N - 1 - k] = s / (float)band_count;
			}

			// Targets (Hz): fixed small set
			const float targets[7] = {2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 8.0f, 10.0f};
			const uint8_t T = (config.modulation_bins <= 7) ? config.modulation_bins : (uint8_t)7;

			float best_pow = 0.0f;
			float best_f = 0.0f;

			for (uint8_t t = 0; t < T; ++t)
			{
				const float f = targets[t];
				const float kf = (2.0f * 3.14159265358979323846f * f) / tick_rate_hz;
				float s_prev = 0.0f, s_prev2 = 0.0f;

				const float coeff = 2.0f * std::cos(kf);
				for (uint8_t n = 0; n < N; ++n)
				{
					const float s = y[n] + coeff * s_prev - s_prev2;
					s_prev2 = s_prev;
					s_prev = s;
				}
				const float re = s_prev - s_prev2 * std::cos(kf);
				const float im = s_prev2 * std::sin(kf);
				const float p = re * re + im * im;
				if (p > best_pow)
				{
					best_pow = p;
					best_f = f;
				}
			}

			return best_f;
		}

		// ---- Evaluate f0 against current frame (with soft reuse penalty) ----
		// Returns (score, centroid_hz, bandwidth_hz, amplitude) and the band list used.
		void eval_f0_with_mask(const CochlearFrame& cur,
			float f0,
			const float* claimed, // per-band [0..1] soft penalty
			float reuse_penalty,
			float& out_score,
			float& out_centroid_hz,
			float& out_bandwidth_hz,
			float& out_amplitude,
			uint16_t* out_bands, // filled with used band indices
			uint8_t& out_band_count) const
		{
			const uint16_t nb = (uint16_t)config.num_bands;
			const uint8_t H = config.max_harmonics;
			const float tol_cents = config.harmonic_tolerance_cents;

			float num = 0.0f;
			float den = 0.0f;
			float wsum_f = 0.0f;
			float wsum = 0.0f;

			out_band_count = 0;

			for (uint8_t h = 1; h <= H; ++h)
			{
				const float target = f0 * (float)h;
				if (target >= config.fmax_hz)
					break;

				const uint16_t idx = band_index_for_hz(target);

				// Search a tiny neighbourhood
				for (int di = -1; di <= 1; ++di)
				{
					const int j = (int)idx + di;
					if (j < 0 || j >= (int)nb)
						continue;

					const float bin_hz = band_hz((uint16_t)j);
					const float cents_off = std::fabs(cents_between(target, bin_hz));
					const float env = cur.envelope[j];

					if (env <= 0.0f)
						continue;

					// Soft harmonic gate and soft reuse penalty
					float within = (cents_off <= tol_cents) ? (1.0f - (cents_off / tol_cents)) : 0.0f;
					if (within <= 0.0f)
						continue;

					const float penalty = 1.0f - reuse_penalty * clampf(claimed[j], 0.0f, 1.0f);
					const float contrib = env * within * penalty;

					den += env;
					num += contrib;

					wsum_f += contrib * bin_hz;
					wsum += contrib;

					// record band index (unique-ish; we only have up to 3*H entries)
					if (out_band_count < (uint8_t)(3 * H))
					{
						out_bands[out_band_count++] = (uint16_t)j;
					}
				}
			}

			out_amplitude = num;
			out_score = (den > 1e-9f) ? (num / den) : 0.0f;

			if (wsum > 1e-9f)
			{
				out_centroid_hz = wsum_f / wsum;

				float var = 0.0f;
				for (uint8_t k = 0; k < out_band_count; ++k)
				{
					const uint16_t j = out_bands[k];
					const float bin_hz = band_hz(j);
					// Use current frame’s effective contribution as weight proxy
					const float d = bin_hz - out_centroid_hz;
					// approximate with current envelope weight
					const float w = cur.envelope[j];
					var += w * d * d;
				}
				const float wsum2 = wsum + 1e-9f;
				out_bandwidth_hz = std::sqrt(var / wsum2);
			}
			else
			{
				out_centroid_hz = 0.0f;
				out_bandwidth_hz = 0.0f;
			}
		}

		// Acquire or create a track slot (nearest in pitch)
		uint8_t acquire_track(float pitch_hz, double ts)
		{
			uint8_t best = 255;
			float best_dp = 1e9f;

			for (uint8_t i = 0; i < State::MaxTracks; ++i)
			{
				if (!state.tracks[i].active)
					continue;
				const float dp = std::fabs(state.tracks[i].pitch_hz - pitch_hz);
				if (dp < best_dp)
				{
					best_dp = dp;
					best = i;
				}
			}

			if (best != 255 && best_dp < 80.0f)
				return best;

			for (uint8_t i = 0; i < State::MaxTracks; ++i)
			{
				if (!state.tracks[i].active)
				{
					state.tracks[i].active = true;
					state.tracks[i].id = state.next_track_id++;
					state.tracks[i].last_timestamp = ts;
					return i;
				}
			}

			// Replace stalest
			uint8_t stalest = 0;
			double oldest = state.tracks[0].last_timestamp;
			for (uint8_t i = 1; i < State::MaxTracks; ++i)
			{
				if (state.tracks[i].last_timestamp < oldest)
				{
					oldest = state.tracks[i].last_timestamp;
					stalest = i;
				}
			}
			state.tracks[stalest].active = true;
			state.tracks[stalest].id = state.next_track_id++;
			state.tracks[stalest].last_timestamp = ts;
			return stalest;
		}

		// ===== Tick =====

		void tick(const TickInfo& tick_info)
		{
			const CochlearFrame& cur = inputs.cochlear_frame;
			outputs.source_candidates.clear();

			// Lazy init ring
			if (!state.initialised)
			{
				std::memset(&state, 0, sizeof(state));
				state.initialised = true;
			}

			// Guard num_bands
			if (config.num_bands == 0 || config.num_bands > MaxBands)
				config.num_bands = (config.num_bands == 0) ? 1 : MaxBands;

			// Record history (for coherence/modulation)
			push_history(cur);

			const uint16_t nb = (uint16_t)config.num_bands;
			state.reset_claims(nb);

			// Quick gate on overall energy
			if (frame_energy(cur) < config.min_amplitude)
			{
				// Age out stale tracks
				for (uint8_t i = 0; i < State::MaxTracks; ++i)
					if (state.tracks[i].active && (cur.timestamp - state.tracks[i].last_timestamp > 0.3))
						state.tracks[i].active = false;
				return;
			}

			// Scan f0 candidates on a geometric grid
			struct Cand
			{
				float f0, score, centroid, bw, amp, coh;
				float mod;
				uint8_t band_count;
				uint16_t bands[3 * 32];
			};
			FixedVector<Cand, 16> pool;

			const float min_f = clampf(config.f0_min_hz, config.fmin_hz, config.fmax_hz);
			const float max_f = clampf(config.f0_max_hz, config.fmin_hz, config.fmax_hz);
			const float step = 1.04f;

			// Greedy K selection with deconfliction
			const uint8_t K = (config.max_sources < 8) ? config.max_sources : (uint8_t)8;

			for (uint8_t ksel = 0; ksel < K; ++ksel)
			{
				// Evaluate all f0 against the *current* claimed mask
				Cand best{};
				best.score = 0.0f;
				bool have = false;

				for (float f0 = min_f; f0 <= max_f; f0 *= step)
				{
					Cand c{};
					c.f0 = f0;
					c.band_count = 0;
					eval_f0_with_mask(cur, f0, state.claimed_energy, config.reuse_penalty, c.score, c.centroid, c.bw, c.amp, c.bands, c.band_count);
					if (c.score < config.min_harmonicity || c.amp < config.min_amplitude)
						continue;

					// Temporal coherence over involved bands
					float group_env = 0.0f;
					c.coh = temporal_coherence_score(c.bands, c.band_count, tick_info.tick_rate_hz, group_env);

					// Combine sieve score + temporal coherence (simple product)
					const float combined = c.score * (0.5f + 0.5f * c.coh);

					if (!have || combined > (best.score * (0.5f + 0.5f * best.coh)))
					{
						best = c;
						have = true;
					}
				}

				if (!have)
					break;

				// Modulation rate over grouped envelope (uses history)
				best.mod = estimate_modulation_rate_hz(tick_info.tick_rate_hz, best.bands, best.band_count);

				// Apply soft claim to bands to discourage reuse by subsequent K selections
				for (uint8_t b = 0; b < best.band_count; ++b)
				{
					const uint16_t j = best.bands[b];
					// Claim proportional to current envelope (clamped 0..1)
					const float e = clampf(cur.envelope[j], 0.0f, 1.0f);
					state.claimed_energy[j] = clampf(state.claimed_energy[j] + 0.6f * e, 0.0f, 1.0f);
				}

				// Keep
				if (pool.size() < pool.capacity())
					pool.add(best);
			}

			// Emit with EMA via tracks
			for (size_t i = 0; i < pool.size(); ++i)
			{
				const Cand& c = pool[i];
				const uint8_t tix = acquire_track(c.f0, cur.timestamp);
				auto& t = state.tracks[tix];

				const float a = clampf(config.smooth_alpha, 0.0f, 1.0f);
				if (t.last_timestamp <= 0.0)
				{
					t.pitch_hz = c.f0;
					t.amplitude = c.amp;
					t.centroid_hz = c.centroid;
					t.bandwidth_hz = c.bw;
					t.harmonicity = c.score;
					t.temporal_coherence = c.coh;
					t.modulation_rate = c.mod;
				}
				else
				{
					t.pitch_hz = a * c.f0 + (1.0f - a) * t.pitch_hz;
					t.amplitude = a * c.amp + (1.0f - a) * t.amplitude;
					t.centroid_hz = a * c.centroid + (1.0f - a) * t.centroid_hz;
					t.bandwidth_hz = a * c.bw + (1.0f - a) * t.bandwidth_hz;
					t.harmonicity = a * c.score + (1.0f - a) * t.harmonicity;
					t.temporal_coherence = a * c.coh + (1.0f - a) * t.temporal_coherence;
					// For modulation rate, pick latest if far apart; otherwise EMA
					if (std::fabs(t.modulation_rate - c.mod) > 1.5f)
						t.modulation_rate = c.mod;
					else
						t.modulation_rate = a * c.mod + (1.0f - a) * t.modulation_rate;
				}
				t.last_timestamp = cur.timestamp;

				SourceCandidate out{};
				out.id = t.id;
				out.pitch_hz = t.pitch_hz;
				out.harmonicity = clampf(t.harmonicity * (0.5f + 0.5f * t.temporal_coherence), 0.0f, 1.0f);
				out.amplitude = t.amplitude;
				out.centroid_freq_hz = t.centroid_hz;
				out.bandwidth_hz = t.bandwidth_hz;
				out.temporal_coherence = clampf(t.temporal_coherence, 0.0f, 1.0f);
				out.modulation_rate = t.modulation_rate;

				outputs.source_candidates.add(out);
				if (outputs.source_candidates.size() >= outputs.source_candidates.capacity())
					break;
			}

			// Retire stale tracks
			for (uint8_t i = 0; i < State::MaxTracks; ++i)
			{
				if (!state.tracks[i].active)
					continue;
				if (cur.timestamp - state.tracks[i].last_timestamp > 0.3)
					state.tracks[i].active = false;
			}
		}
	};
} // namespace robotick
