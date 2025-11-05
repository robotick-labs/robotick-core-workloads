// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0
//
// TemporalGroupingV0.cpp

#include "robotick/systems/auditory/TemporalGroupingV0.h"

#include "robotick/api.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iterator> // for std::size
#include <vector>

namespace robotick
{
	ROBOTICK_REGISTER_STRUCT_BEGIN(TemporalGroupingV0Settings)
	ROBOTICK_STRUCT_FIELD(TemporalGroupingV0Settings, float, fmin_hz)
	ROBOTICK_STRUCT_FIELD(TemporalGroupingV0Settings, float, fmax_hz)
	ROBOTICK_STRUCT_FIELD(TemporalGroupingV0Settings, uint16_t, num_bands)
	ROBOTICK_STRUCT_FIELD(TemporalGroupingV0Settings, float, f0_min_hz)
	ROBOTICK_STRUCT_FIELD(TemporalGroupingV0Settings, float, f0_max_hz)
	ROBOTICK_STRUCT_FIELD(TemporalGroupingV0Settings, uint8_t, max_harmonics)
	ROBOTICK_STRUCT_FIELD(TemporalGroupingV0Settings, float, harmonic_tolerance_cents)
	ROBOTICK_STRUCT_FIELD(TemporalGroupingV0Settings, float, min_harmonicity)
	ROBOTICK_STRUCT_FIELD(TemporalGroupingV0Settings, float, min_amplitude)
	ROBOTICK_STRUCT_FIELD(TemporalGroupingV0Settings, float, reuse_penalty)
	ROBOTICK_STRUCT_FIELD(TemporalGroupingV0Settings, uint8_t, history_frames)
	ROBOTICK_STRUCT_FIELD(TemporalGroupingV0Settings, float, coherence_min_window_s)
	ROBOTICK_STRUCT_FIELD(TemporalGroupingV0Settings, uint8_t, modulation_bins)
	ROBOTICK_STRUCT_FIELD(TemporalGroupingV0Settings, bool, infer_missing_fundamental)
	ROBOTICK_STRUCT_FIELD(TemporalGroupingV0Settings, float, smooth_alpha)
	ROBOTICK_STRUCT_FIELD(TemporalGroupingV0Settings, uint8_t, max_sources)
	ROBOTICK_REGISTER_STRUCT_END(TemporalGroupingV0Settings)

	// Clamp 'value' to the inclusive range [min_value, max_value].
	float TemporalGroupingV0::clampf(float value, float min_value, float max_value)
	{
		if (value < min_value)
			return min_value;
		if (value > max_value)
			return max_value;
		return value;
	}

	// Musical cents between two frequencies (1200 cents per octave).
	// Returns a large value if either input is non-positive.
	float TemporalGroupingV0::cents_between(float base_hz, float target_hz)
	{
		if (base_hz <= 0.0f || target_hz <= 0.0f)
			return 1200.0f;
		const float ratio = target_hz / base_hz;
		return 1200.0f * (float)std::log2((double)ratio);
	}

	// Find the nearest band index to 'query_hz' given monotonically increasing band centers.
	// Returns -1 if no reasonable index can be found (should be rare if inputs are valid).
	int TemporalGroupingV0::band_index_for_hz(const float* band_center_hz, int num_bands, float query_hz)
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
	float TemporalGroupingV0::band_local_width_hz(const float* band_center_hz, int num_bands, int band_index)
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

	// Local effective width (cents) for a band, taken as the average gap to its neighbors.
	float TemporalGroupingV0::band_width_cents(float* band_center_hz, int num_bands, int band_index)
	{
		const float width_hz = band_local_width_hz(band_center_hz, num_bands, band_index);
		const float center_hz = band_center_hz[band_index];
		return std::fabs(cents_between(center_hz, center_hz + 0.5f * width_hz)) * 2.0f;
	}

	/**
	 * @brief Finds the best matching band index for a harmonic frequency, considering ±1 neighbors.
	 *
	 * @param target_hz           The target harmonic frequency (Hz).
	 * @param band_center_hz      Array of band center frequencies [num_bands].
	 * @param envelope            Per-band envelope/energy values [num_bands].
	 * @param num_bands           Number of valid bands in the arrays.
	 * @param tolerance_cents     Max deviation in cents allowed for a band to match the target.
	 * @param out_within_tolerance Output: tolerance weight (0.0–1.0) for selected band.
	 * @param out_envelope        Output: raw envelope value of selected band.
	 * @return int                Index of the best matching band, or -1 if none found within tolerance.
	 */

	int TemporalGroupingV0::find_best_band_for_harmonic(float target_hz,
		const float* band_center_hz,
		const float* envelope,
		int num_bands,
		float tolerance_cents,
		float& out_within_tolerance,
		float& out_envelope)
	{
		int best_band = -1;
		out_within_tolerance = 0.0f;
		out_envelope = 0.0f;

		const int nearest = band_index_for_hz(band_center_hz, num_bands, target_hz);
		if (nearest < 0)
			return -1;

		for (int offset = -1; offset <= 1; ++offset)
		{
			const int i = nearest + offset;
			if (i < 0 || i >= num_bands)
				continue;

			const float f_bin = band_center_hz[i];
			const float env = envelope[i];
			if (env <= 0.0f)
				continue;

			const float cents = std::fabs(cents_between(target_hz, f_bin));
			const float band_width = band_width_cents(const_cast<float*>(band_center_hz), num_bands, i);
			const float hard_cutoff = 0.5f * band_width; // Give full value inside band width

			float within = 0.0f;
			if (cents <= hard_cutoff)
			{
				within = 1.0f;
			}
			else if (cents <= tolerance_cents)
			{
				// Linearly decay from 1.0 → 0.0 starting after hard cutoff
				const float fade_range = tolerance_cents - hard_cutoff + 1e-6f;
				within = 1.0f - (cents - hard_cutoff) / fade_range;
			}

			if (within > out_within_tolerance || (within == out_within_tolerance && env > out_envelope))
			{
				best_band = i;
				out_within_tolerance = within;
				out_envelope = env;
			}
		}

		return best_band;
	}

	/**
	 * @brief Computes the contribution score of a band given tolerance, reuse penalty, and envelope.
	 *
	 * @param envelope        The raw envelope (energy) at this band.
	 * @param within_tolerance Tolerance weight from 0.0 (rejected) to 1.0 (exact match).
	 * @param claimed_fraction Value from 0.0 (unused) to 1.0 (fully claimed by another source).
	 * @param config          TemporalGroupingV0 config containing reuse penalty.
	 * @return float          Final contribution score (can be 0 if fully rejected).
	 */
	float TemporalGroupingV0::compute_band_contribution(
		float envelope, float within_tolerance, float claimed_fraction, const TemporalGroupingV0Settings& config)
	{
		const float clamped = clampf(claimed_fraction, 0.0f, 1.0f);
		const float reuse_penalty = 1.0f - config.reuse_penalty * clamped;
		return envelope * within_tolerance * reuse_penalty;
	}

	/**
	 * @brief Determines whether a detected group passes the missing-fundamental test.
	 *
	 * @param config                Configuration settings (including inference toggle).
	 * @param fundamental_hit       True if h=1 was directly detected.
	 * @param harmonic_energy       Array of per-harmonic contributions [32].
	 * @param band_count            Number of accepted bands in the result.
	 * @param early_energy_fraction Fraction of energy from h1 and h2 (vs total).
	 * @param early_hits            Number of harmonics accepted in h=1..2 range.
	 * @return bool                 True if the candidate should be accepted; false otherwise.
	 */
	bool TemporalGroupingV0::passes_missing_fundamental_gate(const TemporalGroupingV0Settings& config,
		bool fundamental_hit,
		const float* harmonic_energy,
		uint8_t band_count,
		float early_energy_fraction,
		uint8_t early_hits)
	{
		(void)early_hits;

		if (!config.infer_missing_fundamental)
			return fundamental_hit;

		const bool has_h2 = (harmonic_energy[2] > 0.0f);
		const bool has_h3 = (harmonic_energy[3] > 0.0f);
		const bool multiple = (band_count >= 2);
		const bool strong = (early_energy_fraction >= 0.45f);

		return (has_h2 && has_h3 && multiple && strong);
	}

	/**
	 * @brief Modulates harmonicity based on the frequency span of accepted bands.
	 *
	 * Widely spaced harmonics across the spectrum are stronger evidence of a valid f₀
	 * than tightly clustered peaks. This function increases the harmonicity score
	 * based on how much the accepted band frequencies span out, relative to average bin width.
	 *
	 * @param band_center_hz Array of band center frequencies [num_bands].
	 * @param num_bands      Number of valid entries in band_center_hz.
	 * @param envelope       Per-band envelope values [num_bands].
	 * @param out            Result struct with accepted band indices and current harmonicity.
	 */
	void TemporalGroupingV0::apply_span_based_harmonicity_adjustment(const float* band_center_hz, int num_bands, TemporalGroupingV0Result& out)
	{
		if (out.band_count < 2)
			return;

		float min_freq = band_center_hz[out.bands[0]];
		float max_freq = min_freq;
		float sum_widths = 0.0f;

		for (uint8_t i = 0; i < out.band_count; ++i)
		{
			const int idx = (int)out.bands[i];
			const float f = band_center_hz[idx];
			if (f < min_freq)
				min_freq = f;
			if (f > max_freq)
				max_freq = f;

			sum_widths += band_local_width_hz(band_center_hz, num_bands, idx);
		}

		const float span_hz = max_freq - min_freq;
		const float avg_width = sum_widths / (float)out.band_count;
		const float span_target = 2.5f * avg_width;

		// Compute factor from 0.0 to 1.0 based on how much the span exceeds typical bin widths
		const float span_factor = clampf(span_hz / (span_target + 1e-9f), 0.0f, 1.0f);

		// Boost harmonicity using this span factor
		out.harmonicity *= (0.5f + 0.5f * span_factor);
	}

	// --- Peak clustering + peak-based f0 evaluation ------------------------------
	// Minimal, generic, and fast. No new public API needed.

	struct Peak
	{
		int l = 0, r = 0;
		int i_max = 0;
		float amp_sum = 0.0f;
		float amp_max = 0.0f;
		float centroid_hz = 0.0f;
		float claimed_avg = 0.0f;
		float edge_left_hz = 0.0f;	// NEW: inclusive left edge in Hz
		float edge_right_hz = 0.0f; // NEW: inclusive right edge in Hz
	};

	inline bool is_local_peak_bin(const float* env, int n, int i)
	{
		const float c = env[i];
		const float left = (i > 0) ? env[i - 1] : -INFINITY;
		const float right = (i + 1 < n) ? env[i + 1] : -INFINITY;
		return c >= left && c >= right && (c > left || c > right);
	}

	static void extract_peaks(const float* band_center_hz,
		const float* envelope,
		const float* claimed, // optional
		int num_bands,
		float min_abs_amp,
		float rel_min_frac, // relative to global peak
		int grow_left = 1,
		int grow_right = 1,
		std::vector<Peak>* out_peaks = nullptr)
	{
		out_peaks->clear();
		if (num_bands <= 0)
			return;

		// Global peak for relative threshold
		float global_peak = 0.0f;
		for (int i = 0; i < num_bands; ++i)
			global_peak = std::max(global_peak, envelope[i]);

		const float amp_thresh = std::max(min_abs_amp, rel_min_frac * global_peak);
		if (amp_thresh <= 0.0f)
			return;

		for (int i = 0; i < num_bands; ++i)
		{
			if (envelope[i] < amp_thresh)
				continue;
			if (!is_local_peak_bin(envelope, num_bands, i))
				continue;

			// Grow a compact cluster around the local max while values decrease
			int l = i, r = i;
			// expand left
			for (int step = 0; step < grow_left; ++step)
			{
				if (l <= 0)
					break;
				if (envelope[l - 1] <= envelope[l])
					--l;
				else
					break;
			}
			// expand right
			for (int step = 0; step < grow_right; ++step)
			{
				if (r >= num_bands - 1)
					break;
				if (envelope[r + 1] <= envelope[r])
					++r;
				else
					break;
			}

			// Integrate the cluster
			float wsum = 0.0f, fwsum = 0.0f, csum = 0.0f;
			for (int j = l; j <= r; ++j)
			{
				const float w = envelope[j];
				wsum += w;
				fwsum += w * band_center_hz[j];
				if (claimed)
					csum += claimed[j];
			}
			if (wsum < amp_thresh)
				continue; // filter tiny clusters

			Peak p;
			p.l = l;
			p.r = r;
			p.i_max = i;
			p.amp_sum = wsum;
			p.amp_max = envelope[i];
			p.centroid_hz = (wsum > 0.0f) ? (fwsum / wsum) : band_center_hz[i];
			p.claimed_avg = (claimed && (r >= l)) ? (csum / float(r - l + 1)) : 0.0f;

			out_peaks->push_back(p);

			// Optional: skip ahead to avoid overlapping peaks (simple NMS)
			i = r;
		}
	}

	// cents_between(f1,f2) already exists in your code; re-use it.

	static int find_best_peak_for_harmonic(
		float target_hz, const std::vector<Peak>& peaks, float tolerance_cents, float& out_within_tol, float& out_amp_sum)
	{
		int best = -1;
		float best_score = -1.0f;

		for (int k = 0; k < (int)peaks.size(); ++k)
		{
			const float cents = std::fabs(TemporalGroupingV0::cents_between(target_hz, peaks[k].centroid_hz));
			if (cents > tolerance_cents)
				continue;

			// Heuristic: prefer closer in cents, then larger amp_sum
			const float closeness = 1.0f - (cents / (tolerance_cents + 1e-9f));
			const float score = closeness * peaks[k].amp_sum;

			if (score > best_score)
			{
				best_score = score;
				best = k;
				out_within_tol = closeness; // 0..1
				out_amp_sum = peaks[k].amp_sum;
			}
		}
		return best;
	}

	// -----------------------------------------------------------------------------
	// Peak-based eval_f0_with_mask
	// -----------------------------------------------------------------------------

	void TemporalGroupingV0::eval_f0_with_mask(const float* band_center_hz,
		const float* envelope,
		const float* claimed,
		int num_bands,
		const TemporalGroupingV0Settings& config,
		float f0,
		TemporalGroupingV0Result& out,
		float* harmonic_energy_out)
	{
		out = TemporalGroupingV0Result{};
		if (num_bands <= 0 || f0 <= 0.0f)
			return;

		float harmonic_energy[32] = {};
		const uint8_t max_harmonics = std::min<uint8_t>(config.max_harmonics, 31);
		const float tolerance_cents = config.harmonic_tolerance_cents;

		// 1) Build peak list once for this frame (generic; no overfit)
		std::vector<Peak> peaks;
		peaks.reserve(64);

		// Tunables (could be lifted to config later)
		const float rel_min = 0.02f; // 2% of global peak
		const int grow_left = 2;	 // allow a slightly wider cluster
		const int grow_right = 2;

		extract_peaks(band_center_hz,
			envelope,
			claimed,
			num_bands,
			/*min_abs_amp=*/config.min_amplitude,
			/*rel_min_frac=*/rel_min,
			grow_left,
			grow_right,
			&peaks);

		if (peaks.empty())
			return;

		// Keep track of which peaks were consumed (so a harmonic can't reuse a nearby peak)
		std::vector<uint8_t> peak_used(peaks.size(), 0);

		float energy_sum = 0.0f;
		float unique_energy = 0.0f;
		float centroid_sum = 0.0f;
		float weight_sum = 0.0f;

		bool fundamental_hit = false;
		uint8_t early_hits = 0;
		uint8_t band_count = 0;

		// 2) Harmonic scan (match to peaks, not bins)
		for (uint8_t h = 1; h <= max_harmonics; ++h)
		{
			const float target_hz = f0 * h;
			if (target_hz >= config.fmax_hz)
				break;

			float within = 0.0f, amp_sum = 0.0f;
			int pk = find_best_peak_for_harmonic(target_hz, peaks, tolerance_cents, within, amp_sum);
			if (pk < 0)
				continue;
			if (peak_used[pk])
				continue;

			// Score contribution using your existing function:
			const float claim = peaks[pk].claimed_avg;
			const float contrib = compute_band_contribution(amp_sum, within, claim, config);
			if (contrib <= config.min_amplitude)
				continue;

			peak_used[pk] = 1;

			// Choose a representative band index for outputs (use peak max)
			int repr_idx = peaks[pk].i_max;
			if (band_count < (uint8_t)std::size(out.bands))
				out.bands[band_count++] = (uint16_t)repr_idx;

			energy_sum += contrib;
			centroid_sum += contrib * peaks[pk].centroid_hz;
			weight_sum += contrib;
			unique_energy += amp_sum;

			if (h < 32)
				harmonic_energy[h] += contrib;

			if (h == 1 && contrib > config.min_amplitude)
				++early_hits, fundamental_hit = true;
			if (h == 2 && contrib > config.min_amplitude)
				++early_hits;
		}

		out.band_count = band_count;
		out.amplitude = energy_sum;

		// 3) Early-gate and quality metrics
		if (band_count == 0 || energy_sum <= 0.0f)
			return;

		const float early_energy = harmonic_energy[1] + harmonic_energy[2];
		const float early_frac = early_energy / (energy_sum + 1e-12f);

		if (!passes_missing_fundamental_gate(config, fundamental_hit, harmonic_energy, band_count, early_frac, early_hits))
		{
			out.band_count = 0;
			return;
		}
		if (early_frac < 0.20f || early_hits < 1)
		{
			out.band_count = 0;
			return;
		}

		out.harmonicity = (unique_energy > 1e-9f) ? (energy_sum / unique_energy) : 0.0f;

		if (weight_sum > 1e-9f)
		{
			out.centroid_hz = centroid_sum / weight_sum;

			// Bandwidth using original envelope weights around centroid (optional)
			float var_sum = 0.0f;
			for (uint8_t i = 0; i < band_count; ++i)
			{
				const int idx = out.bands[i];
				const float f = band_center_hz[idx];
				const float df = f - out.centroid_hz;
				const float w = envelope[idx];
				var_sum += w * df * df;
			}
			out.bandwidth_hz = std::sqrt(var_sum / (weight_sum + 1e-9f));
		}

		if (band_count >= 2)
			apply_span_based_harmonicity_adjustment(band_center_hz, num_bands, out);

		if (harmonic_energy_out)
			std::memcpy(harmonic_energy_out, harmonic_energy, sizeof(float) * 32);

		out.f0_hz = f0;
	}

	// Measure temporal coherence of a group of bands over recent history.
	// Returns 0..1 where higher means the per-band envelopes co-vary with the group's mean envelope over time.
	float TemporalGroupingV0::temporal_coherence_score(const float* const* history_envelopes,
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
	float TemporalGroupingV0::estimate_modulation_rate_hz(const float* const* history_envelopes,
		uint8_t history_count,
		uint8_t /*history_cap*/,
		const uint16_t* selected_band_indices,
		uint8_t selected_band_count,
		int num_bands,
		float tick_rate_hz,
		const TemporalGroupingV0Settings& config)
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

		// Detrend: remove DC offset (mean) from the group envelope
		float sum = 0.0f;
		for (uint8_t i = 0; i < frame_count; ++i)
			sum += group_envelope_series[i];
		const float mean = sum / (float)frame_count;
		for (uint8_t i = 0; i < frame_count; ++i)
			group_envelope_series[i] -= mean;

		// Candidate AM rates (Hz) common in speech rhythm
		static const float kTargetRatesHz[7] = {2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 8.0f, 10.0f};
		const uint8_t num_target_bins = (config.modulation_bins <= 7) ? config.modulation_bins : (uint8_t)7;

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
