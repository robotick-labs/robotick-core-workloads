// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#include "robotick/systems/auditory/HarmonicPitch.h"

#include "robotick/api.h"

namespace robotick
{
	ROBOTICK_REGISTER_STRUCT_BEGIN(HarmonicPitchSettings)
	ROBOTICK_STRUCT_FIELD(HarmonicPitchSettings, float, min_amplitude)
	ROBOTICK_STRUCT_FIELD(HarmonicPitchSettings, float, min_peak_falloff_norm)
	ROBOTICK_REGISTER_STRUCT_END(HarmonicPitchSettings)

	static bool harmonic_amplitudes_to_string(const void* data, char* out_buffer, size_t buffer_size)
	{
		const HarmonicAmplitudes* buf = static_cast<const HarmonicAmplitudes*>(data);
		if (!buf || !out_buffer || buffer_size < 32)
			return false;

		// Format: <HarmonicAmplitudes(size/capacity)>
		int written = snprintf(out_buffer, buffer_size, "<HarmonicAmplitudes(%zu/%zu)>", buf->size(), buf->capacity());
		return written > 0 && static_cast<size_t>(written) < buffer_size;
	}

	static bool harmonic_amplitudes_from_string(const char*, void*)
	{
		// Read-only string representation, parsing not supported
		return false;
	}

	ROBOTICK_REGISTER_PRIMITIVE(HarmonicAmplitudes, harmonic_amplitudes_to_string, harmonic_amplitudes_from_string);

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

		FixedVector<Peak, 32> peaks;
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
		const float ratio_tolerance = std::pow(2.0f, cents_tolerance / 1200.0f);

		float best_score = -1.0f;
		float best_f0 = 0.0f;
		int best_match_count = 0;
		FixedVector<float, harmonic_pitch::MaxHarmonics> best_amplitudes;

		for (const Peak& candidate : peaks)
		{
			const float candidate_f0 = candidate.frequency;
			FixedVector<float, harmonic_pitch::MaxHarmonics> harmonics;

			int max_non_zero_harmonic_id = 0;
			int match_count = 0;
			float score = 0.0f;

			for (size_t harmonic_id = 1; harmonic_id <= harmonic_pitch::MaxHarmonics; ++harmonic_id)
			{
				const float expected_freq = candidate_f0 * harmonic_id;
				float best_amp = 0.0f;
				float best_cents_error = cents_tolerance + 1.0f;

				for (const Peak& peak : peaks)
				{
					const float ratio = peak.frequency / expected_freq;
					if (ratio < (1.0f / ratio_tolerance) || ratio > ratio_tolerance)
						continue;

					const float cents_error = 1200.0f * std::abs(std::log2(ratio));
					if (cents_error < best_cents_error)
					{
						best_cents_error = cents_error;
						best_amp = peak.amplitude;
					}
				}

				if (best_amp > 0.0f)
				{
					match_count++;
					score += best_amp;
					max_non_zero_harmonic_id = harmonic_id;
				}

				harmonics.add(best_amp);
			}

			const bool is_single_peak = (match_count == 1);
			const bool is_valid_stack = (match_count >= min_stack_matches);

			if (is_valid_stack || (is_single_peak && settings.allow_single_peak_mode))
			{
				const bool is_better = (score > best_score) || (score == best_score && match_count > best_match_count);

				if (is_better)
				{
					best_score = score;
					best_f0 = candidate_f0;
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

		return true;
	}

} // namespace robotick
