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

	static int find_first_peak_value_index(const HarmonicPitchSettings& settings, const AudioBuffer128& envelope, const size_t start_band_id)
	{
		const size_t num_bands = envelope.size();
		if (num_bands < 3 || start_band_id + 2 >= num_bands)
			return -1;

		int candidate_peak_band_id = -1;
		float candidate_peak_value = 0.0f;

		for (size_t band_id = start_band_id; band_id < num_bands; ++band_id)
		{
			const float current_value = std::max(0.0f, envelope[band_id] - settings.min_amplitude);
			if (current_value <= 0.0f)
				continue;

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
						const float back_value = std::max(0.0f, envelope[other_band_id] - settings.min_amplitude);
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
						const int centroid = (rise_index + falloff_index) / 2;
						return centroid;
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

		return -1; // No valid peak found
	}

	int HarmonicPitch::find_strongest_f0_band_id(
		const HarmonicPitchSettings& settings, const AudioBuffer128& centers, const AudioBuffer128& envelope, HarmonicPitchResult& result)
	{
		ROBOTICK_ASSERT(centers.size() == envelope.size());
		(void)centers;
		(void)result;

		const int peak_value_band = find_first_peak_value_index(settings, envelope, 0);
		if (peak_value_band < 0)
		{
			return -1; // nothing found
		}

		ROBOTICK_ASSERT(peak_value_band < (int)envelope.size());

		const float peak_value = envelope[peak_value_band];
		if (peak_value <= settings.min_amplitude)
		{
			return -1; // nothing significant enough found
		}

		result.h1_f0_hz = centers[peak_value_band];
		result.h1_amplitude = peak_value;

		return peak_value_band; // found
	}

} // namespace robotick
