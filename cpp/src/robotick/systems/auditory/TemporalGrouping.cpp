// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#include "robotick/systems/auditory/TemporalGrouping.h"

#include "robotick/api.h"

namespace robotick
{
	ROBOTICK_REGISTER_STRUCT_BEGIN(TemporalGroupingSettings)
	ROBOTICK_STRUCT_FIELD(TemporalGroupingSettings, float, min_amplitude)
	ROBOTICK_STRUCT_FIELD(TemporalGroupingSettings, float, reuse_penalty)
	ROBOTICK_REGISTER_STRUCT_END(TemporalGroupingSettings)

	static int find_peak_value_index(const AudioBuffer128& envelope)
	{
		int best_id = -1;
		float best_value = 0.0f;

		for (size_t band_id = 0; band_id < envelope.size(); ++band_id)
		{
			const float current_value = envelope[band_id];

			if (band_id == 0 || current_value > best_value)
			{
				best_id = band_id;
				best_value = current_value;
			}
		}

		return best_id;
	}

	int TemporalGrouping::find_strongest_f0_band_id(
		const TemporalGroupingSettings& settings, const AudioBuffer128& centers, const AudioBuffer128& envelope, TemporalGroupingResult& result)
	{
		ROBOTICK_ASSERT(centers.size() == envelope.size());
		(void)centers;
		(void)result;

		const int peak_value_band = find_peak_value_index(envelope);
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
