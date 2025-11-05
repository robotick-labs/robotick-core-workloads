// Copyright Robotick
// SPDX-License-Identifier: Apache-2.0
//
// TemporalGroupingV0Workload.cpp  (thin wrapper around robotick::TemporalGroupingV0)

#include "robotick/api.h"
#include "robotick/systems/audio/AudioSystem.h"
#include "robotick/systems/auditory/CochlearFrame.h"
#include "robotick/systems/auditory/SourceCandidate.h"
#include "robotick/systems/auditory/TemporalGroupingV0.h"

#include <cmath>
#include <cstring>
#include <fstream>

namespace robotick
{
	struct TemporalGroupingV0Config
	{
		TemporalGroupingV0Settings settings;
	};

	struct TemporalGroupingV0Inputs
	{
		CochlearFrame cochlear_frame;
	};

	struct TemporalGroupingV0Outputs
	{
		SourceCandidates8 source_candidates;
		SourceCandidate first_source;
	};

	struct TemporalGroupingV0Workload
	{
		TemporalGroupingV0Config config;
		TemporalGroupingV0Inputs inputs;
		TemporalGroupingV0Outputs outputs;

		// History + tracks (data only)
		static constexpr uint16_t MaxBands = 256;
		static constexpr uint8_t MaxHistory = 32;

		struct HistEntry
		{
			float envelope[MaxBands];
			double timestamp = 0.0;
		};
		struct Track
		{
			bool active = false;
			uint8_t id = 0;
			float pitch_hz = 0.0f, amplitude = 0.0f, centroid_hz = 0.0f, bandwidth_hz = 0.0f;
			float harmonicity = 0.0f, temporal_coherence = 0.0f, modulation_rate = 0.0f;
			double last_timestamp = 0.0;
		};

		struct State
		{
			HistEntry history[MaxHistory];
			uint8_t history_count = 0;
			uint8_t history_head = 0;

			float claimed_energy[MaxBands];

			static constexpr uint8_t MaxTracks = 8;
			Track tracks[MaxTracks];
			uint8_t next_track_id = 1;

			void reset_claims(uint16_t num_bands)
			{
				const uint16_t n = (num_bands < MaxBands) ? num_bands : MaxBands;
				for (uint16_t i = 0; i < n; ++i)
					claimed_energy[i] = 0.0f;
			}
		} state;

		static inline float clampf(float v, float lo, float hi)
		{
			if (v < lo)
				return lo;
			if (v > hi)
				return hi;
			return v;
		}

		inline float frame_energy(const CochlearFrame& f) const
		{
			const uint16_t num_bands = (config.settings.num_bands <= MaxBands) ? config.settings.num_bands : MaxBands;
			float s = 0.0f;
			for (uint16_t i = 0; i < num_bands; ++i)
				s += f.envelope[i];
			return s;
		}

		void push_history(const CochlearFrame& f)
		{
			const uint8_t cap = (config.settings.history_frames > MaxHistory) ? MaxHistory : config.settings.history_frames;
			if (cap == 0)
				return;

			state.history_head = (uint8_t)((state.history_head + 1) % cap);
			HistEntry& e = state.history[state.history_head];

			const uint16_t num_bands = (config.settings.num_bands <= MaxBands) ? config.settings.num_bands : MaxBands;
			for (uint16_t i = 0; i < num_bands; ++i)
				e.envelope[i] = f.envelope[i];
			e.timestamp = f.timestamp;

			if (state.history_count < cap)
				++state.history_count;

#if ENABLE_TG_ENVELOPE_LOG
			static std::ofstream log_file("envelope_history.log", std::ios::app);
			if (log_file.is_open())
			{
				// Optional: log timestamp too
				log_file << f.timestamp;
				for (uint16_t i = 0; i < num_bands; ++i)
					log_file << "," << f.envelope[i];
				log_file << "\n";
			}
			static std::ofstream log_file_2("centers_history.log", std::ios::app);
			if (log_file_2.is_open())
			{
				// Optional: log timestamp too
				log_file_2 << f.timestamp;
				for (uint16_t i = 0; i < num_bands; ++i)
					log_file_2 << "," << f.band_center_hz[i];
				log_file_2 << "\n";
			}
#endif // #if ENABLE_TG_ENVELOPE_LOG
		}

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
				if (state.tracks[i].last_timestamp < oldest)
				{
					oldest = state.tracks[i].last_timestamp;
					stalest = i;
				}
			state.tracks[stalest].active = true;
			state.tracks[stalest].id = state.next_track_id++;
			state.tracks[stalest].last_timestamp = ts;
			return stalest;
		}

		void tick(const TickInfo& tick_info)
		{
			const CochlearFrame& current_frame = inputs.cochlear_frame;
			outputs.source_candidates.clear();
			outputs.first_source = SourceCandidate{};

			if (config.settings.num_bands == 0 || config.settings.num_bands > MaxBands)
				config.settings.num_bands = (config.settings.num_bands == 0) ? 1 : MaxBands;

			push_history(current_frame);

			const uint16_t num_bands = config.settings.num_bands;
			state.reset_claims(num_bands);

			// Energy gate
			if (frame_energy(current_frame) < config.settings.min_amplitude)
			{
				for (uint8_t i = 0; i < State::MaxTracks; ++i)
					if (state.tracks[i].active && (current_frame.timestamp - state.tracks[i].last_timestamp > 0.3))
					{
						state.tracks[i].active = false;
					}
				return;
			}

			// Candidate scan (geometric grid)
			struct Cand
			{
				TemporalGroupingV0Result res;
			};
			FixedVector<Cand, 16> pool;

			const float min_f = robotick::TemporalGroupingV0::clampf(config.settings.f0_min_hz, config.settings.fmin_hz, config.settings.fmax_hz);
			const float max_f = robotick::TemporalGroupingV0::clampf(config.settings.f0_max_hz, config.settings.fmin_hz, config.settings.fmax_hz);
			const float step = 1.04f;
			const uint8_t K = (config.settings.max_sources < 8) ? config.settings.max_sources : (uint8_t)8;

			for (uint8_t ksel = 0; ksel < K; ++ksel)
			{
				bool have = false;
				Cand best{};
				float best_combined = 0.0f;

				for (float f0 = min_f; f0 <= max_f; f0 *= step)
				{
					Cand c{};
					robotick::TemporalGroupingV0::eval_f0_with_mask(current_frame.band_center_hz.data(),
						current_frame.envelope.data(),
						state.claimed_energy,
						(int)num_bands,
						config.settings,
						f0,
						c.res,
						nullptr);
					if (c.res.band_count == 0)
						continue;

					const float frameE = frame_energy(current_frame);
					const float min_rel = 0.12f;
					const bool single_ridge_ok = (c.res.band_count == 1 && c.res.harmonicity >= 0.50f);

					if (c.res.amplitude < config.settings.min_amplitude)
						continue;
					if (!single_ridge_ok && c.res.amplitude < min_rel * frameE)
						continue;
					if (c.res.harmonicity < config.settings.min_harmonicity)
						continue;

					// Build history arrays oldest..newest
					float const* hist_ptrs[MaxHistory];
					double ts_arr[MaxHistory];
					const uint8_t N = state.history_count;
					const uint8_t cap = (config.settings.history_frames > MaxHistory) ? MaxHistory : config.settings.history_frames;
					for (uint8_t k = 0; k < N; ++k)
					{
						const uint8_t idx = (uint8_t)((state.history_head + cap - (N - 1 - k)) % cap);
						hist_ptrs[k] = state.history[idx].envelope;
						ts_arr[k] = state.history[idx].timestamp;
					}

					float group_mean = 0.0f;
					const float coh = robotick::TemporalGroupingV0::temporal_coherence_score(
						hist_ptrs, ts_arr, N, cap, c.res.bands, c.res.band_count, (int)num_bands, config.settings.coherence_min_window_s, group_mean);
					c.res.temporal_coherence = robotick::TemporalGroupingV0::clampf(coh, 0.0f, 1.0f);

					const float combined = c.res.harmonicity * (0.5f + 0.5f * c.res.temporal_coherence);
					if (!have || combined > best_combined)
					{
						best = c;
						best_combined = combined;
						have = true;
					}
				}

				if (!have)
					break;

				// Modulation (history)
				float const* hist_ptrs2[MaxHistory];
				const uint8_t N2 = state.history_count;
				const uint8_t cap2 = (config.settings.history_frames > MaxHistory) ? MaxHistory : config.settings.history_frames;
				for (uint8_t k = 0; k < N2; ++k)
				{
					const uint8_t idx = (uint8_t)((state.history_head + cap2 - (N2 - 1 - k)) % cap2);
					hist_ptrs2[k] = state.history[idx].envelope;
				}
				best.res.modulation_rate_hz = robotick::TemporalGroupingV0::estimate_modulation_rate_hz(
					hist_ptrs2, N2, cap2, best.res.bands, best.res.band_count, (int)num_bands, tick_info.tick_rate_hz, config.settings);

				// Soft-claim
				for (uint8_t b = 0; b < best.res.band_count; ++b)
				{
					const uint16_t j = best.res.bands[b];
					const float e = robotick::TemporalGroupingV0::clampf(current_frame.envelope[j], 0.0f, 1.0f);
					state.claimed_energy[j] = robotick::TemporalGroupingV0::clampf(state.claimed_energy[j] + 0.6f * e, 0.0f, 1.0f);
				}

				auto is_duplicate_pitch = [&](float hz)
				{
					for (size_t i = 0; i < pool.size(); ++i)
					{
						const float a = pool[i].res.f0_hz;
						// Use cents or Hz; cents is more scale-robust:
						const float cents = 1200.0f * std::log2(std::max(hz, 1e-6f) / std::max(a, 1e-6f));
						if (std::fabs(cents) < 10.0f) // ~ <= 0.6% difference
							return true;
					}
					return false;
				};

				if (!is_duplicate_pitch(best.res.f0_hz))
				{
					// Soft-claim ALL bins that came back in res.bands (now the entire peak span)
					for (uint8_t b = 0; b < best.res.band_count; ++b)
					{
						const uint16_t j = best.res.bands[b];
						const float e = clampf(current_frame.envelope[j], 0.0f, 1.0f);
						// stronger claim inside same tick so it actually blocks re-picks:
						state.claimed_energy[j] = clampf(state.claimed_energy[j] + 1.0f * e, 0.0f, 1.0f);
					}
					if (pool.size() < pool.capacity())
						pool.add(best);
				}
			}

			// Emit via simple EMA
			for (size_t i = 0; i < pool.size(); ++i)
			{
				const auto& r = pool[i].res;
				const uint8_t tix = acquire_track(r.f0_hz, current_frame.timestamp);
				auto& t = state.tracks[tix];

				const float a = clampf(config.settings.smooth_alpha, 0.0f, 1.0f);
				if (t.last_timestamp <= 0.0)
				{
					t.pitch_hz = r.f0_hz;
					t.amplitude = r.amplitude;
					t.centroid_hz = r.centroid_hz;
					t.bandwidth_hz = r.bandwidth_hz;
					t.harmonicity = r.harmonicity;
					t.temporal_coherence = r.temporal_coherence;
					t.modulation_rate = r.modulation_rate_hz;
				}
				else
				{
					t.pitch_hz = a * r.f0_hz + (1.0f - a) * t.pitch_hz;
					t.amplitude = a * r.amplitude + (1.0f - a) * t.amplitude;
					t.centroid_hz = a * r.centroid_hz + (1.0f - a) * t.centroid_hz;
					t.bandwidth_hz = a * r.bandwidth_hz + (1.0f - a) * t.bandwidth_hz;
					t.harmonicity = a * r.harmonicity + (1.0f - a) * t.harmonicity;
					t.temporal_coherence = a * r.temporal_coherence + (1.0f - a) * t.temporal_coherence;
					if (std::fabs(t.modulation_rate - r.modulation_rate_hz) > 1.5f)
						t.modulation_rate = r.modulation_rate_hz;
					else
						t.modulation_rate = a * r.modulation_rate_hz + (1.0f - a) * t.modulation_rate;
				}
				t.last_timestamp = current_frame.timestamp;

				SourceCandidate out{};
				out.pitch_hz = t.pitch_hz;
				out.harmonicity = clampf(t.harmonicity * (0.5f + 0.5f * t.temporal_coherence), 0.0f, 1.0f);
				out.amplitude = t.amplitude;
				out.centroid_freq_hz = t.centroid_hz;
				out.bandwidth_hz = t.bandwidth_hz;
				out.temporal_coherence = clampf(t.temporal_coherence, 0.0f, 1.0f);
				out.modulation_rate = t.modulation_rate;

				if (i == 0)
					outputs.first_source = out;
				outputs.source_candidates.add(out);
				if (outputs.source_candidates.size() >= outputs.source_candidates.capacity())
					break;
			}

			// Retire stale tracks
			for (uint8_t i = 0; i < State::MaxTracks; ++i)
				if (state.tracks[i].active && (current_frame.timestamp - state.tracks[i].last_timestamp > 0.3))
					state.tracks[i].active = false;
		}
	};

} // namespace robotick