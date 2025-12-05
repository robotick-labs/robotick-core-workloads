// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#include "robotick/systems/auditory/HarmonicPitch.h"

#include "robotick/api.h"
#include "robotick/framework/math/Abs.h"
#include "robotick/framework/math/LogExp.h"
#include "robotick/framework/math/Pow.h"

namespace robotick
{
	ROBOTICK_REGISTER_FIXED_VECTOR(HarmonicAmplitudes, float);

	ROBOTICK_REGISTER_STRUCT_BEGIN(HarmonicPitchSettings)
	ROBOTICK_STRUCT_FIELD(HarmonicPitchSettings, float, min_amplitude)
	ROBOTICK_STRUCT_FIELD(HarmonicPitchSettings, float, min_peak_falloff_norm)
	ROBOTICK_REGISTER_STRUCT_END(HarmonicPitchSettings)

	ROBOTICK_REGISTER_STRUCT_BEGIN(HarmonicPitchResult)
	ROBOTICK_STRUCT_FIELD(HarmonicPitchResult, float, h1_f0_hz)
	ROBOTICK_STRUCT_FIELD(HarmonicPitchResult, HarmonicAmplitudes, harmonic_amplitudes)
	ROBOTICK_REGISTER_STRUCT_END(HarmonicPitchResult)

	/**
	 * @brief Find the first valid spectral peak in a cochlear envelope using an "island" detection strategy.
	 *
	 * This function scans forward from the given start band, tracking the highest envelope value
	 * (above a minimum threshold) as a candidate peak. Once a sufficient **falloff** is observed
	 * (i.e. the envelope drops by a fraction of the peak height), the function retroactively checks
	 * for a corresponding **rise** before the peak — confirming the peak is bounded by lower values
	 * on both sides (like a perceptual island).
	 *
	 * If both conditions are met, the function returns the **centroid index** between the rise and falloff bounds.
	 *
	 * This approach is robust to noise and smoothing effects and avoids premature peak detection due
	 * to local fluctuations.
	 *
	 * @param settings               Thresholds and parameters for peak detection.
	 * @param envelope               The cochlear envelope to scan (typically smoothed, compressed magnitude).
	 * @param start_band_id          Band index to begin scanning from.
	 * @return The centroid index of the first valid peak, or -1 if none found.
	 */

	struct PeakRegion
	{
		int rise_band_id = -1;
		int peak_band_id = -1;
		int fall_band_id = -1;

		bool is_valid() const { return (rise_band_id >= 0 && peak_band_id >= 0 && fall_band_id >= 0); }

		bool get_approx_frequency_and_amplitude(
			const AudioBuffer128& centers, const AudioBuffer128& envelope, float& out_frequency, float& out_amplitude) const
		{
			if (!is_valid())
			{
				return false;
			}

			ROBOTICK_ASSERT(rise_band_id >= 0 && rise_band_id < (int)envelope.size());
			ROBOTICK_ASSERT(peak_band_id >= 0 && peak_band_id < (int)envelope.size());
			ROBOTICK_ASSERT(fall_band_id >= 0 && fall_band_id < (int)envelope.size());

			// Initial base-quantised estimate:
			out_frequency = centers[peak_band_id];
			out_amplitude = envelope[peak_band_id];

			// Refine our estimate if possible:
			float weighted_sum = 0.0f;
			float total_weight = 0.0f;

			for (int band_id = rise_band_id; band_id <= fall_band_id; ++band_id)
			{
				const float val = envelope[band_id];
				weighted_sum += val * centers[band_id];
				total_weight += val;
			}

			if (total_weight > 0.0f)
			{
				out_frequency = weighted_sum / total_weight;
			}

			return true; // found!
		}
	};

	static PeakRegion find_first_peak_region(const HarmonicPitchSettings& settings, const AudioBuffer128& envelope, const size_t start_band_id)
	{
		const size_t num_bands = envelope.size();
		PeakRegion region;

		if (num_bands < 3 || start_band_id + 2 >= num_bands)
			return region;

		int candidate_peak_band_id = -1;
		float candidate_peak_value = 0.0f;

		for (size_t band_id = start_band_id; band_id < num_bands; ++band_id)
		{
			const float current_value = max(0.0f, envelope[band_id] - settings.min_amplitude);

			// Found new candidate peak
			if (current_value > candidate_peak_value)
			{
				candidate_peak_band_id = static_cast<int>(band_id);
				candidate_peak_value = current_value;
				continue;
			}

			// Check for valid falloff
			if (candidate_peak_band_id >= 0)
			{
				const float required_drop = settings.min_peak_falloff_norm * candidate_peak_value;

				if ((candidate_peak_value - current_value) >= required_drop)
				{
					// Scan backward to find a preceeding corresponding rise
					int rise_index = candidate_peak_band_id;
					for (int other_band_id = candidate_peak_band_id - 1; other_band_id >= 0; --other_band_id)
					{
						const float back_value = max(0.0f, envelope[other_band_id] - settings.min_amplitude);
						if ((candidate_peak_value - back_value) >= required_drop)
						{
							rise_index = other_band_id;
							break;
						}
					}

					// Only valid if rise found
					if (rise_index < candidate_peak_band_id)
					{
						const int falloff_index = static_cast<int>(band_id);
						region.rise_band_id = rise_index;
						region.peak_band_id = candidate_peak_band_id;
						region.fall_band_id = falloff_index;
						return region;
					}
					else
					{
						// No valid rise — discard peak and keep scanning
						candidate_peak_band_id = -1;
						candidate_peak_value = 0.0f;
					}
				}
			}
		}

		return region; // Defaults to -1s if not found
	}

	bool HarmonicPitch::find_harmonic_features(
		const HarmonicPitchSettings& settings, const AudioBuffer128& centers, const AudioBuffer128& envelope, HarmonicPitchResult& result)
	{
		ROBOTICK_ASSERT(centers.size() == envelope.size());

		result.h1_f0_hz = 0.0f;
		result.harmonic_amplitudes.clear();

		// ----------------------------------------
		// Step 1: Extract all envelope peaks
		// ----------------------------------------

		struct Peak
		{
			float frequency;
			float amplitude;
		};

		FixedVector<Peak, AudioBuffer128::capacity()> peaks;
		size_t band_id = 0;

		while (band_id < envelope.size())
		{
			const PeakRegion region = find_first_peak_region(settings, envelope, band_id);
			if (!region.is_valid())
				break;

			float freq = 0.0f;
			float ampl = 0.0f;
			if (region.get_approx_frequency_and_amplitude(centers, envelope, freq, ampl))
			{
				peaks.add({freq, ampl});
			}

			band_id = region.fall_band_id + 1;
		}

		if (peaks.size() == 0)
			return false;

		// ----------------------------------------
		// Step 2: Try each peak as candidate f0
		// ----------------------------------------

		static_assert(result.harmonic_amplitudes.capacity() == harmonic_pitch::MaxHarmonics);

		const int min_stack_matches = 3; // we can't declare a clear harmonic pattern without at least 3 matches
		const float cents_tolerance = settings.harmonic_tolerance_cents;
		const float ratio_tolerance = robotick::pow(2.0f, cents_tolerance / 1200.0f);

		float best_score = -1.0f;
		float best_f0 = 0.0f;
		int best_match_count = 0;
		FixedVector<float, harmonic_pitch::MaxHarmonics> best_amplitudes;

		// For each peak in the spectrum, consider it as a possible fundamental frequency (f0)
		for (const Peak& candidate : peaks)
		{
			const float candidate_f0 = candidate.frequency;
			FixedVector<float, harmonic_pitch::MaxHarmonics> harmonics;

			int max_non_zero_harmonic_id = 0;
			int match_count = 0;
			float score = 0.0f;

			// Try to find matching harmonic peaks at integer multiples of this candidate f0
			for (size_t harmonic_id = 1; harmonic_id <= harmonics.capacity(); ++harmonic_id)
			{
				const float expected_freq = candidate_f0 * harmonic_id;

				// Track the best match for this harmonic across all peaks
				float best_amp = 0.0f;
				float best_cents_error = cents_tolerance + 1.0f;

				for (const Peak& peak : peaks)
				{
					const float ratio = peak.frequency / expected_freq;

					// Skip if frequency mismatch is too large (based on ratio tolerance)
					if (ratio < (1.0f / ratio_tolerance) || ratio > ratio_tolerance)
						continue;

					// Convert ratio to musical 'cents' error (1 semitone = 100 cents)
					const float cents_error = 1200.0f * robotick::abs(robotick::log2(ratio));

					// Keep the closest (in cents) peak within tolerance
					if (cents_error < best_cents_error)
					{
						best_cents_error = cents_error;
						best_amp = peak.amplitude;
					}
				}

				// Record amplitude for this harmonic (0 if unmatched)
				harmonics.add(best_amp);

				if (best_amp > 0.0f)
				{
					match_count++;							// Count how many harmonics were matched
					score += best_amp;						// Accumulate score (favor stronger matches)
					max_non_zero_harmonic_id = harmonic_id; // Track furthest matched harmonic
				}
			}

			// Decide whether this candidate is worth considering:
			// - valid stack: enough harmonics match (e.g. voiced sound)
			// - or single peak, allowed by setting (e.g. whistle or sine tone)
			const bool is_single_peak = (match_count == 1);
			const bool is_valid_stack = (match_count >= min_stack_matches);

			if (is_valid_stack || (is_single_peak && settings.allow_single_peak_mode))
			{
				// Prefer higher score, or equal score with more matched harmonics
				const bool is_better = (score > best_score) || (score == best_score && match_count > best_match_count);

				if (is_better)
				{
					best_score = score;
					best_f0 = candidate_f0;

					// Copy harmonic amplitudes up to last matched harmonic
					best_amplitudes.set(harmonics.data(), max_non_zero_harmonic_id);
					best_match_count = match_count;
				}
			}
		}

		// ----------------------------------------
		// Step 3: Accept result (if any)
		// ----------------------------------------

		if (best_score <= 0.0f)
			return false;

		result.h1_f0_hz = best_f0;
		result.harmonic_amplitudes = best_amplitudes;

		// ------------------------------------------------------------------------------------------
		// Step 4: Fill in any harmonics not detected as peaks, with the current band-envelope value
		// ------------------------------------------------------------------------------------------

		for (size_t harmonic_id = 1; harmonic_id <= result.harmonic_amplitudes.size(); ++harmonic_id)
		{
			// Skip any already-filled amplitudes (i.e. those already set reliably from peaks)
			if (result.harmonic_amplitudes[harmonic_id - 1] > 0.0f)
				continue;

			const float harmonic_freq = best_f0 * harmonic_id;

			// Find the band closest to the harmonic frequency
			int closest_band_id = -1;
			float closest_distance = 1e9f;

			for (size_t band_id = 0; band_id < centers.size(); ++band_id)
			{
				const float dist = robotick::abs(centers[band_id] - harmonic_freq);
				if (dist < closest_distance)
				{
					closest_distance = dist;
					closest_band_id = static_cast<int>(band_id);
				}
			}

			if (closest_band_id >= 0 && closest_band_id < static_cast<int>(envelope.size()))
			{
				result.harmonic_amplitudes[harmonic_id - 1] = envelope[closest_band_id] > settings.min_amplitude ? envelope[closest_band_id] : 0.0f;
			}
		}

		return true;
	}

	bool HarmonicPitch::try_continue_previous_result(const HarmonicPitchSettings& settings,
		const AudioBuffer128& centers,
		const AudioBuffer128& envelope,
		const HarmonicPitchResult& prev_result,
		HarmonicPitchResult& result)
	{
		if (prev_result.h1_f0_hz <= 0.0f)
			return false;

		// Step 1: find band index closest to previous f0
		const size_t num_bands = centers.size();
		int prev_f0_band = -1;
		float min_dist = 1e9f;

		for (size_t i = 0; i < num_bands; ++i)
		{
			const float dist = robotick::abs(centers[i] - prev_result.h1_f0_hz);
			if (dist < min_dist)
			{
				min_dist = dist;
				prev_f0_band = static_cast<int>(i);
			}
		}

		if (prev_f0_band < 0 || prev_f0_band >= static_cast<int>(num_bands))
			return false;

		// Step 2: check if we’re still "inside the white snake" at this band
		if (envelope[prev_f0_band] < settings.min_amplitude)
			return false;

		// Step 3: walk outward in both directions to find the extent of this snake
		int start_band = prev_f0_band;
		int end_band = prev_f0_band;

		while (start_band > 0 && envelope[start_band - 1] >= settings.min_amplitude)
			--start_band;

		while (end_band + 1 < static_cast<int>(num_bands) && envelope[end_band + 1] >= settings.min_amplitude)
			++end_band;

		// Step 4: compute centroid within this band range
		float weighted_sum = 0.0f;
		float total_weight = 0.0f;

		for (int i = start_band; i <= end_band; ++i)
		{
			const float amp = envelope[i];
			weighted_sum += centers[i] * amp;
			total_weight += amp;
		}

		if (total_weight <= 0.0f)
			return false;

		const float new_f0 = weighted_sum / total_weight;

		// Step 5: remeasure harmonic amplitudes using updated f0
		FixedVector<float, harmonic_pitch::MaxHarmonics> harmonics;
		int strong_count = 0;

		for (size_t harmonic_id = 1; harmonic_id <= harmonic_pitch::MaxHarmonics; ++harmonic_id)
		{
			const float harmonic_freq = new_f0 * harmonic_id;

			// Find closest band
			int closest_band = -1;
			float min_distance = 1e9f;
			for (size_t i = 0; i < num_bands; ++i)
			{
				const float dist = robotick::abs(centers[i] - harmonic_freq);
				if (dist < min_distance)
				{
					min_distance = dist;
					closest_band = static_cast<int>(i);
				}
			}

			float amp = 0.0f;
			if (closest_band >= 0 && closest_band < static_cast<int>(envelope.size()))
			{
				amp = envelope[closest_band];
				if (amp >= settings.min_amplitude)
					strong_count++;
			}

			harmonics.add(amp);
		}

		if (strong_count < 2)
			return false;

		// Accept continuation
		result.h1_f0_hz = new_f0;
		result.harmonic_amplitudes = harmonics;
		return true;
	}

	bool HarmonicPitch::find_or_continue_harmonic_features(const HarmonicPitchSettings& settings,
		const AudioBuffer128& centers,
		const AudioBuffer128& envelope,
		const HarmonicPitchResult& prev_result,
		HarmonicPitchResult& out_result)
	{
		HarmonicPitchResult fresh;
		HarmonicPitchResult continued;

		// Try detecting a new harmonic stack from scratch
		const bool fresh_ok = find_harmonic_features(settings, centers, envelope, fresh);

		// Try continuing the previous f0 using nearby envelope structure
		const bool continued_ok = try_continue_previous_result(settings, centers, envelope, prev_result, continued);

		// If neither succeeded, give up
		if (!fresh_ok && !continued_ok)
			return false;

		// If only one succeeded, use it directly
		if (fresh_ok && !continued_ok)
		{
			out_result = fresh;
			return true;
		}
		else if (!fresh_ok && continued_ok)
		{
			out_result = continued;
			return true;
		}
		else
		{
			// Both succeeded — check if their f0 values are similar (within cents threshold)
			const float cents_diff = 1200.0f * robotick::abs(robotick::log2(fresh.h1_f0_hz / continued.h1_f0_hz));

			if (cents_diff < settings.harmonic_tolerance_cents)
			{
				// If similar, merge their amplitude profiles into one stronger result
				out_result.h1_f0_hz = fresh.h1_f0_hz;
				out_result.harmonic_amplitudes.clear();

				const size_t max_allowed = out_result.harmonic_amplitudes.capacity();
				const size_t num_harmonics = min(max(fresh.harmonic_amplitudes.size(), continued.harmonic_amplitudes.size()), max_allowed);

				for (size_t i = 0; i < num_harmonics; ++i)
				{
					const float fresh_amp = (i < fresh.harmonic_amplitudes.size()) ? fresh.harmonic_amplitudes[i] : 0.0f;
					const float continued_amp = (i < continued.harmonic_amplitudes.size()) ? continued.harmonic_amplitudes[i] : 0.0f;
					const float out_amp = max(fresh_amp, continued_amp);

					// Only include strong enough harmonics in merged result
					out_result.harmonic_amplitudes.add((out_amp > settings.min_amplitude) ? out_amp : 0.0f);
				}
			}
			else
			{
				// If f0 values disagree, choose whichever has stronger cumulative amplitude
				float fresh_score = 0.0f, continued_score = 0.0f;
				for (float amp : fresh.harmonic_amplitudes)
					fresh_score += amp;
				for (float amp : continued.harmonic_amplitudes)
					continued_score += amp;

				out_result = (fresh_score >= continued_score) ? fresh : continued;
			}

			return true;
		}
	}

} // namespace robotick
