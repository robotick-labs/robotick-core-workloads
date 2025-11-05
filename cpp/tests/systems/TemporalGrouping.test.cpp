// Copyright Robotick
// SPDX-License-Identifier: Apache-2.0

#include "robotick/systems/auditory/TemporalGrouping.h"

#include <catch2/catch_all.hpp>

#include <cmath>
#include <cstring>
#include <vector>

// real-world data kept in seperate file for clarity
#include "TemporalGrouping_data.cpp"

namespace robotick::test
{
	static int find_closest_frequency_center_index(const AudioBuffer128& centers, const float query_freq_hz)
	{
		int best = -1;
		float bd = 1e30f;
		for (size_t i = 0; i < centers.size(); ++i)
		{
			float d = std::fabs(centers[i] - query_freq_hz);
			if (d < bd)
			{
				bd = d;
				best = i;
			}
		}
		return best;
	}

	TEST_CASE("Unit/Audio/TemporalGrouping")
	{
		SECTION("Detects only the true fundamental in real-world 1200Hz sine-wave envelope profile")
		{
			TemporalGroupingSettings config;
			config.min_amplitude = 0.1f;
			config.reuse_penalty = 0.45f;

			const int num_bands = 128;
			const float expected_f0_hz = 1200.0f;

			// Use real-world envelope profile (128 values)

			static_assert(sizeof(s_real_1200hz_sinewave_centers) / sizeof(float) == num_bands, "Expected 128 values in real_centers");
			static_assert(sizeof(s_real_1200hz_sinewave_envelope) / sizeof(float) == num_bands, "Expected 128 values in real_profile");

			AudioBuffer128 centers;
			centers.set(s_real_1200hz_sinewave_centers, num_bands);

			AudioBuffer128 envelope;
			envelope.set(s_real_1200hz_sinewave_envelope, num_bands);

			// === Find closest band to 1200 Hz ===
			const int ix1200 = find_closest_frequency_center_index(centers, 1200.0f);
			REQUIRE(ix1200 >= 0);

			TemporalGroupingResult result{};
			const int found_band_id = TemporalGrouping::find_strongest_f0_band_id(config, centers, envelope, result);

			CHECK(result.h1_amplitude > 0.5f);
			CHECK(result.h1_f0_hz == Catch::Approx(expected_f0_hz).margin(5.0f));
			REQUIRE(found_band_id == ix1200);
		}
	}

}; // namespace robotick::test
