// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0
//
// TemporalGrouping.cpp

#include "robotick/systems/auditory/TemporalGrouping.h"
#include <cmath>
#include <cstring>

namespace robotick
{
	// Clamp 'value' to the inclusive range [min_value, max_value].
	float TemporalGrouping::clampf(float value, float min_value, float max_value)
	{
		if (value < min_value)
			return min_value;
		if (value > max_value)
			return max_value;
		return value;
	}

	// Musical cents between two frequencies (1200 cents per octave).
	// Returns a large value if either input is non-positive.
	float TemporalGrouping::cents_between(float base_hz, float target_hz)
	{
		if (base_hz <= 0.0f || target_hz <= 0.0f)
			return 1200.0f;
		const float ratio = target_hz / base_hz;
		return 1200.0f * (float)std::log2((double)ratio);
	}

	// Find the nearest band index to 'query_hz' given monotonically increasing band centers.
	// Returns -1 if no reasonable index can be found (should be rare if inputs are valid).
	int TemporalGrouping::band_index_for_hz(const float* band_center_hz, int num_bands, float query_hz)
	{
		if (num_bands <= 1)
			return -1;
		if (query_hz <= band_center_hz[0])
			return 0;
		if (query_hz >= band_center_hz[num_bands - 1])
			return (num_bands - 1);

		for (int i = 0; i < num_bands - 1; ++i)
		{
			const float f0 = band_center_hz[i];
			const float f1 = band_center_hz[i + 1];
			if (query_hz >= f0 && query_hz < f1)
				return ((query_hz - f0) < (f1 - query_hz)) ? i : (i + 1);
		}
		return -1;
	}

	// Local effective width (Hz) for a band, taken as the average gap to its neighbors.
	float TemporalGrouping::band_local_width_hz(const float* band_center_hz, int num_bands, int band_index)
	{
		if (num_bands <= 1)
			return 1.0f;
		if (band_index <= 0)
			return 0.5f * (band_center_hz[1] - band_center_hz[0]);
		if (band_index >= num_bands - 1)
			return 0.5f * (band_center_hz[num_bands - 1] - band_center_hz[num_bands - 2]);

		const float left_gap = band_center_hz[band_index] - band_center_hz[band_index - 1];
		const float right_gap = band_center_hz[band_index + 1] - band_center_hz[band_index];
		return 0.5f * (left_gap + right_gap);
	}

	// Evaluate the match of a candidate f0 (fundamental) against the envelope spectrum, with a "claimed" mask to penalize reuse.
	// - band_center_hz: center frequency of each analysis band
	// - envelope:       per-band energy/envelope (non-negative)
	// - claimed:        per-band 0..1 mask indicating how much energy has already been used by another source (optional)
	// - num_bands:      number of valid bands
	// - cfg:            temporal grouping configuration
	// - f0:             candidate fundamental to evaluate (Hz)
	// - out:            result structure (filled on success)
	// - E_h_out:        optional per-harmonic contribution array [0..31]
	void TemporalGrouping::eval_f0_with_mask(const float* band_center_hz,
		const float* envelope,
		const float* claimed,
		int num_bands,
		const TemporalGroupingConfig& cfg,
		float f0,
		TemporalGroupingResult& out,
		float* harmonic_energy_out)
	{
		out = TemporalGroupingResult{};
		if (num_bands <= 0 || f0 <= 0.0f)
			return;

		// Guard sizes for small automatic arrays
		const int kMaxBandsLocal = 256;
		const int local_band_capacity = (num_bands < kMaxBandsLocal) ? num_bands : kMaxBandsLocal;

		// Limit harmonics we will consider (array below supports up to 31 harmonics explicitly)
		uint8_t max_harmonics = cfg.max_harmonics;
		if (max_harmonics > 31)
			max_harmonics = 31;

		const float harmonic_tol_cents = cfg.harmonic_tolerance_cents;

		// Accumulators for amplitude and centroid
		float accepted_energy_weighted = 0.0f;	 // sum of (band_energy * within_tolerance * reuse_penalty)
		float unique_energy_denominator = 0.0f;	 // sum of raw band energies for accepted bins (for harmonicity)
		float centroid_weighted_freq_sum = 0.0f; // sum of (contrib * band_center_hz)
		float total_weight = 0.0f;				 // sum of contrib

		// Track if we reused a band (to avoid double-counting); only for locally addressable bands
		bool band_used_locally[kMaxBandsLocal];
		for (int i = 0; i < local_band_capacity; ++i)
			band_used_locally[i] = false;

		// Per-harmonic accepted energy contributions (E_h[h])
		float harmonic_energy[32];
		for (int i = 0; i < 32; ++i)
			harmonic_energy[i] = 0.0f;

		uint8_t out_band_count = 0;
		bool fundamental_hit = false;		   // whether we accepted h=1
		uint8_t early_harmonic_hits_count = 0; // among h=1..2 (first two harmonics)

		// Scan harmonics at integer multiples of f0
		for (uint8_t h = 1; h <= max_harmonics; ++h)
		{
			const float harmonic_target_hz = f0 * (float)h;
			if (harmonic_target_hz >= cfg.fmax_hz)
				break;

			const int nearest_band_index = band_index_for_hz(band_center_hz, num_bands, harmonic_target_hz);
			if (nearest_band_index < 0)
				continue;

			// Search the nearest band and its immediate neighbors for the best in-tolerance hit
			int best_band_index = -1;
			float best_within_tolerance = 0.0f;
			float best_band_envelope = 0.0f;

			for (int neighbor_offset = -1; neighbor_offset <= 1; ++neighbor_offset)
			{
				const int band_index = nearest_band_index + neighbor_offset;
				if (band_index < 0 || band_index >= num_bands)
					continue;

				const float bin_hz = band_center_hz[band_index];
				const float band_envelope = envelope[band_index];
				if (band_envelope <= 0.0f)
					continue;

				const float cents_off = std::fabs(cents_between(harmonic_target_hz, bin_hz));
				const float within_tolerance = (cents_off <= harmonic_tol_cents) ? (1.0f - (cents_off / (harmonic_tol_cents + 1e-12f))) : 0.0f;

				if (within_tolerance <= 0.0f)
					continue;

				// Prefer higher within_tolerance; tiebreak by larger band_envelope
				if (within_tolerance > best_within_tolerance || (within_tolerance == best_within_tolerance && band_envelope > best_band_envelope))
				{
					best_within_tolerance = within_tolerance;
					best_band_envelope = band_envelope;
					best_band_index = band_index;
				}
			}

			if (best_band_index < 0)
				continue;

			// Avoid picking the same band twice within this evaluation (local guard)
			if (best_band_index < local_band_capacity && band_used_locally[best_band_index])
				continue;

			// Penalize bands that are already "claimed" by other sources
			const float claimed_fraction = claimed ? claimed[best_band_index] : 0.0f; // 0..1
			const float reuse_penalty = 1.0f - cfg.reuse_penalty * clampf(claimed_fraction, 0.0f, 1.0f);

			const float contribution = best_band_envelope * best_within_tolerance * reuse_penalty;

			if (best_band_index < local_band_capacity)
				band_used_locally[best_band_index] = true;

			// Record which bands were accepted (for downstream consumers / debugging)
			if (out_band_count < (uint8_t)(sizeof(out.bands) / sizeof(out.bands[0])))
				out.bands[out_band_count++] = (uint16_t)best_band_index;

			accepted_energy_weighted += contribution;
			centroid_weighted_freq_sum += contribution * band_center_hz[best_band_index];
			total_weight += contribution;

			unique_energy_denominator += best_band_envelope;

			// Track per-harmonic contribution and early-harmonic evidence
			harmonic_energy[h] += contribution;
			if (h == 1 && contribution > 0.0f)
				fundamental_hit = true;
			if (h <= 2 && contribution > 0.0f)
				++early_harmonic_hits_count;
		}

		out.band_count = out_band_count;
		out.amplitude = accepted_energy_weighted;

		// Reject if we had no valid contributions
		if (accepted_energy_weighted <= 0.0f || out_band_count == 0)
		{
			out.band_count = 0;
			return;
		}

		// Early-harmonic evidence (helps reject octave/half-octave mistakes)
		const float early_energy_sum = harmonic_energy[1] + harmonic_energy[2];
		const float early_energy_fraction = early_energy_sum / (accepted_energy_weighted + 1e-12f);

		// If we *don't* infer missing fundamentals, insist we actually saw h=1
		if (!cfg.infer_missing_fundamental && !fundamental_hit)
		{
			out.band_count = 0;
			return;
		}

		// If we *do* infer missing fundamentals, require strong early-harmonic structure
		if (cfg.infer_missing_fundamental && !fundamental_hit)
		{
			const bool has_h2 = (harmonic_energy[2] > 0.0f);
			const bool has_h3 = (harmonic_energy[3] > 0.0f);
			const bool multiple_bands = (out_band_count >= 2);
			const bool strong_early = (early_energy_fraction >= 0.45f);

			if (!(has_h2 && has_h3 && multiple_bands && strong_early))
			{
				out.band_count = 0;
				return;
			}
		}

		// Minimal early evidence gate
		if (early_energy_fraction < 0.20f || early_harmonic_hits_count < 1)
		{
			out.band_count = 0;
			return;
		}

		// Harmonicity = (accepted weighted energy) / (sum of raw unique energies)
		out.harmonicity = (unique_energy_denominator > 1e-9f) ? (accepted_energy_weighted / unique_energy_denominator) : 0.0f;

		// Spectral centroid and bandwidth over the accepted bands
		if (total_weight > 1e-9f)
		{
			out.centroid_hz = centroid_weighted_freq_sum / total_weight;

			float weighted_variance = 0.0f;
			for (uint8_t k = 0; k < out_band_count; ++k)
			{
				const int band_index = (int)out.bands[k];
				const float f = band_center_hz[band_index];
				const float df = f - out.centroid_hz;
				const float w = envelope[band_index];
				weighted_variance += w * df * df;
			}
			out.bandwidth_hz = std::sqrt(weighted_variance / (total_weight + 1e-9f));
		}

		// Span-based adjustment: wideness of accepted bands vs their typical widths
		if (out_band_count >= 2)
		{
			float min_freq = band_center_hz[out.bands[0]];
			float max_freq = min_freq;
			float sum_local_widths = 0.0f;

			for (uint8_t k = 0; k < out_band_count; ++k)
			{
				const int band_index = (int)out.bands[k];
				const float f = band_center_hz[band_index];
				if (f < min_freq)
					min_freq = f;
				if (f > max_freq)
					max_freq = f;
				sum_local_widths += band_local_width_hz(band_center_hz, num_bands, band_index);
			}

			const float span_hz = max_freq - min_freq;
			const float mean_local_width = sum_local_widths / (float)out_band_count;

			// Heuristic: compare span to a scaled average width to modulate harmonicity
			const float span_target = 2.5f * mean_local_width;
			const float span_factor_0_1 = clampf(span_hz / (span_target + 1e-9f), 0.0f, 1.0f);

			// Note: this *increases* harmonicity with span_factor; keep logic as-is.
			out.harmonicity *= (0.5f + 0.5f * span_factor_0_1);
		}

		// Optional per-harmonic output
		if (harmonic_energy_out)
			for (int i = 0; i < 32; ++i)
				harmonic_energy_out[i] = harmonic_energy[i];

		out.f0_hz = f0;
	}

	// Measure temporal coherence of a group of bands over recent history.
	// Returns 0..1 where higher means the per-band envelopes co-vary with the group's mean envelope over time.
	float TemporalGrouping::temporal_coherence_score(const float* const* history_envelopes,
		const double* timestamps,
		uint8_t history_count,
		uint8_t /*history_cap*/,
		const uint16_t* selected_band_indices,
		uint8_t selected_band_count,
		int num_bands,
		float min_window_seconds,
		float& out_group_env_mean)
	{
		out_group_env_mean = 0.0f;
		if (!history_envelopes || !timestamps || history_count < 3 || selected_band_count == 0 || num_bands <= 0)
			return 0.0f;

		const uint8_t frame_count = history_count;
		const double newest_time = timestamps[frame_count - 1];
		const double oldest_time = timestamps[0];
		if ((newest_time - oldest_time) < (double)min_window_seconds)
			return 0.0f;

		// Per-frame mean envelope across the group's bands
		float mean_envelope_over_bands[64];
		for (uint8_t k = 0; k < frame_count; ++k)
		{
			float sum_band_env = 0.0f;
			const float* env = history_envelopes[k];
			for (uint8_t b = 0; b < selected_band_count; ++b)
			{
				const int band_index = (int)selected_band_indices[b];
				if (band_index >= 0 && band_index < num_bands)
					sum_band_env += env[band_index];
			}
			mean_envelope_over_bands[k] = sum_band_env / (float)selected_band_count;
			out_group_env_mean += mean_envelope_over_bands[k];
		}
		out_group_env_mean /= (float)frame_count;

		// Reject if the group is essentially flat (near-zero variance)
		float overall_mean = 0.0f;
		for (uint8_t k = 0; k < frame_count; ++k)
			overall_mean += mean_envelope_over_bands[k];
		overall_mean /= (float)frame_count;

		float mean_variance = 0.0f;
		for (uint8_t k = 0; k < frame_count; ++k)
		{
			const float dm = mean_envelope_over_bands[k] - overall_mean;
			mean_variance += dm * dm;
		}
		if (mean_variance < 1e-10f)
			return 0.0f;

		// Average Pearson correlation between each band's envelope and the group's mean envelope
		float correlation_sum = 0.0f;
		uint8_t correlation_count = 0;

		for (uint8_t b = 0; b < selected_band_count; ++b)
		{
			float band_series[64];
			for (uint8_t k = 0; k < frame_count; ++k)
			{
				const int band_index = (int)selected_band_indices[b];
				band_series[k] = (band_index >= 0 && band_index < num_bands) ? history_envelopes[k][band_index] : 0.0f;
			}

			float band_mean = 0.0f, group_mean = 0.0f;
			for (uint8_t k = 0; k < frame_count; ++k)
			{
				band_mean += band_series[k];
				group_mean += mean_envelope_over_bands[k];
			}
			band_mean /= (float)frame_count;
			group_mean /= (float)frame_count;

			float numerator = 0.0f, denom_band = 0.0f, denom_group = 0.0f;
			for (uint8_t k = 0; k < frame_count; ++k)
			{
				const float dx = band_series[k] - band_mean;
				const float dg = mean_envelope_over_bands[k] - group_mean;
				numerator += dx * dg;
				denom_band += dx * dx;
				denom_group += dg * dg;
			}

			if (denom_band < 1e-10f || denom_group < 1e-10f)
				continue;

			const float denom = std::sqrt(denom_band * denom_group) + 1e-9f;
			const float r = (denom > 0.0f) ? (numerator / denom) : 0.0f;

			// Map Pearson r (-1..+1) to 0..1
			correlation_sum += (r * 0.5f + 0.5f);
			++correlation_count;
		}

		if (correlation_count == 0)
			return 0.0f;
		return correlation_sum / (float)correlation_count;
	}

	// Estimate a coarse modulation (AM) rate in Hz of the group's envelope over the recent history.
	// Uses a Goertzel pass over a small set of target frequencies (2..10 Hz speech-related syllabic rates).
	float TemporalGrouping::estimate_modulation_rate_hz(const float* const* history_envelopes,
		uint8_t history_count,
		uint8_t /*history_cap*/,
		const uint16_t* selected_band_indices,
		uint8_t selected_band_count,
		int num_bands,
		float tick_rate_hz,
		const TemporalGroupingConfig& cfg)
	{
		if (!history_envelopes || history_count < 6 || selected_band_count == 0 || num_bands <= 0 || tick_rate_hz <= 0.0f)
			return 0.0f;

		const uint8_t frame_count = history_count;

		// Group-average envelope vs time
		float group_envelope_series[64];
		for (uint8_t k = 0; k < frame_count; ++k)
		{
			float sum_band_env = 0.0f;
			for (uint8_t b = 0; b < selected_band_count; ++b)
			{
				const int band_index = (int)selected_band_indices[b];
				if (band_index >= 0 && band_index < num_bands)
					sum_band_env += history_envelopes[k][band_index];
			}
			group_envelope_series[k] = sum_band_env / (float)selected_band_count;
		}

		// Candidate AM rates (Hz) common in speech rhythm
		static const float kTargetRatesHz[7] = {2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 8.0f, 10.0f};
		const uint8_t num_target_bins = (cfg.modulation_bins <= 7) ? cfg.modulation_bins : (uint8_t)7;

		float best_power = 0.0f;
		float best_rate_hz = 0.0f;

		for (uint8_t t = 0; t < num_target_bins; ++t)
		{
			const float freq_hz = kTargetRatesHz[t];
			const float omega = (2.0f * 3.14159265358979323846f * freq_hz) / tick_rate_hz;

			// Goertzel recurrence
			float s_prev = 0.0f, s_prev2 = 0.0f;
			const float coeff = 2.0f * std::cos(omega);

			for (uint8_t n = 0; n < frame_count; ++n)
			{
				const float s = group_envelope_series[n] + coeff * s_prev - s_prev2;
				s_prev2 = s_prev;
				s_prev = s;
			}

			const float re = s_prev - s_prev2 * std::cos(omega);
			const float im = s_prev2 * std::sin(omega);
			const float power = re * re + im * im;

			if (power > best_power)
			{
				best_power = power;
				best_rate_hz = freq_hz;
			}
		}

		return best_rate_hz;
	}

} // namespace robotick
