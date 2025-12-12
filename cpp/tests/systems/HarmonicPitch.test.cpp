// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#include "robotick/systems/auditory/HarmonicPitch.h"

#include "robotick/framework/math/Abs.h"
#include "robotick/framework/math/LogExp.h"
#include "robotick/framework/math/Trig.h"

#include "robotick/framework/math/Pow.h"

#include <catch2/catch_all.hpp>

#include <cmath>
#include <cstring>
#include <vector>

// real-world data kept in seperate file for clarity
#include "HarmonicPitch_data.cpp"

namespace robotick::test
{
	static int find_closest_frequency_center_index(const AudioBuffer128& centers, const float query_freq_hz)
	{
		int best = -1;
		float bd = 1e30f;
		for (size_t i = 0; i < centers.size(); ++i)
		{
			float d = robotick::abs(centers[i] - query_freq_hz);
			if (d < bd)
			{
				bd = d;
				best = i;
			}
		}
		return best;
	}

	static void stamp_gaussian_peak(AudioBuffer128& envelope, const AudioBuffer128& centers, float peak_hz, float amplitude, float sigma_cents)
	{
		for (size_t i = 0; i < centers.size(); ++i)
		{
			const float cents = 1200.0f * robotick::log2(centers[i] / peak_hz);
			const float gauss = robotick::exp(-0.5f * (cents * cents) / (sigma_cents * sigma_cents));
			envelope[i] = robotick::max(envelope[i], amplitude * gauss);
		}
	}

	TEST_CASE("Unit/Audio/HarmonicPitch")
	{
		SECTION("Detects only the true fundamental in real-world 1200Hz sine-wave envelope profile")
		{
			HarmonicPitchSettings config;
			config.min_amplitude = 0.1f;

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

			HarmonicPitchResult result{};
			const bool success = HarmonicPitch::find_harmonic_features(config, centers, envelope, result);

			REQUIRE(success);
			CHECK(result.get_h1_amplitude() > 0.5f);
			CHECK(result.h1_f0_hz == Catch::Approx(expected_f0_hz).margin(5.0f));
		}

		SECTION("Detects a modulating pure sine wave (amplitude + frequency)")
		{
			HarmonicPitchSettings config;
			config.min_amplitude = 0.1f;
			config.min_peak_falloff_norm = 0.25f;

			const int num_bands = 128;
			const float fmin = 100.0f;
			const float fmax = 3500.0f;
			const int steps = 64;

			// Linearly spaced frequency bands for test
			AudioBuffer128 centers(num_bands);
			for (int i = 0; i < num_bands; ++i)
			{
				centers[i] = fmin + (fmax - fmin) * float(i) / (num_bands - 1);
			}

			// Modulation params
			const float base_freq = 500.0f;
			const float freq_mod_depth = 100.0f;
			const float amp_mod_depth = 0.1f;

			// Sweep through time-steps to simulate dynamic changes
			for (int t = 0; t < steps; ++t)
			{
				AudioBuffer128 envelope(num_bands);

				const float t_norm = float(t) / (steps - 1);															   // 0 → 1
				float current_freq = base_freq + freq_mod_depth * robotick::sin(t_norm * 2.0f * static_cast<float>(M_PI)); // sine LFO
				float current_amp = 1.0f + amp_mod_depth * robotick::sin(t_norm * 2.0f * static_cast<float>(M_PI));		   // amplitude LFO

				// Add silence at start and end
				if (t < 4 || t > steps - 5)
				{
					current_amp = 0.0f;
				}

				// Fill envelope with Gaussian around current_freq
				for (int i = 0; i < num_bands; ++i)
				{
					const float band_hz = centers[i];
					const float cents = 1200.0f * robotick::log2(band_hz / current_freq);
					const float sigma = 50.0f;
					const float gauss = robotick::exp(-0.5f * (cents * cents) / (sigma * sigma));
					envelope[i] = current_amp * gauss;
				}

				HarmonicPitchResult result{};

				if (current_amp > 0.1f)
				{
					const bool success = HarmonicPitch::find_harmonic_features(config, centers, envelope, result);
					CHECK(success); // something found
					CHECK(result.get_h1_amplitude() > 0.3f);
					CHECK(result.h1_f0_hz == Catch::Approx(current_freq).margin(5.0f));
				}
				else
				{
					const bool success = HarmonicPitch::find_harmonic_features(config, centers, envelope, result);
					CHECK(!success); // nothing found
					CHECK(result.get_h1_amplitude() == Catch::Approx(0.0f).margin(0.01f));
				}
			}
		}

		SECTION("Detects modulating harmonic-rich signal (like voice) with fading harmonics")
		{
			HarmonicPitchSettings config;
			config.min_amplitude = 0.05f;
			config.min_peak_falloff_norm = 0.25f;

			const int num_bands = 128;
			const float fmin = 100.0f;
			const float fmax = 3500.0f;
			const int steps = 64;

			AudioBuffer128 centers(num_bands);
			for (int i = 0; i < num_bands; ++i)
			{
				centers[i] = fmin * robotick::pow(fmax / fmin, float(i) / (num_bands - 1));
			}

			const float base_f0 = 220.0f;
			const float freq_wobble_cents = 20.0f;
			const int num_harmonics = 8;

			for (int t = 0; t < steps; ++t)
			{
				AudioBuffer128 envelope(num_bands);

				const float t_norm = float(t) / (steps - 1);
				const float f0 =
					base_f0 * robotick::pow(2.0f, (freq_wobble_cents / 1200.0f) * robotick::sin(t_norm * 2.0f * static_cast<float>(M_PI)));
				const float global_amp = (t < 4 || t > steps - 5) ? 0.0f : 0.7f;

				for (int harmonic_id = 1; harmonic_id <= num_harmonics; ++harmonic_id)
				{
					const float partial_hz = f0 * harmonic_id;
					const float amplitude = global_amp * (1.0f / harmonic_id); // roll-off

					for (int i = 0; i < num_bands; ++i)
					{
						const float cents = 1200.0f * robotick::log2(centers[i] / partial_hz);
						const float sigma = 40.0f;
						const float gauss = robotick::exp(-0.5f * (cents * cents) / (sigma * sigma));
						envelope[i] += amplitude * gauss;
					}
				}

				// Cap envelope to 1.0f
				for (int i = 0; i < num_bands; ++i)
					envelope[i] = robotick::min(envelope[i], 1.0f);

				HarmonicPitchResult result{};

				if (global_amp > 0.1f)
				{
					const bool success = HarmonicPitch::find_harmonic_features(config, centers, envelope, result);

					CHECK(success);
					CHECK(result.h1_f0_hz == Catch::Approx(f0).margin(5.0f));
					CHECK(result.get_h1_amplitude() > 0.2f); // even with smearing, this should be > 0

					CHECK(result.h1_f0_hz >= centers[0]);
					CHECK(result.h1_f0_hz <= centers[num_bands - 1]);

					// Harmonic structure validation
					REQUIRE(result.harmonic_amplitudes.size() == num_harmonics);

					// Check that harmonic amplitudes decrease (or at least don’t increase drastically)
					for (int harmonic_id = 1; harmonic_id < num_harmonics - 1; ++harmonic_id)
					{
						CHECK(result.harmonic_amplitudes[harmonic_id] <= result.harmonic_amplitudes[harmonic_id - 1] + 0.05f);
					}

					// Confirm at least a few harmonics were non-zero
					int nonzero_count = 0;
					for (int harmonic_id = 0; harmonic_id < num_harmonics; ++harmonic_id)
					{
						if (result.harmonic_amplitudes[harmonic_id] > 0.05f)
							++nonzero_count;
					}
					CHECK(nonzero_count >= 3);
				}
				else
				{
					const bool success = HarmonicPitch::find_harmonic_features(config, centers, envelope, result);
					CHECK(!success);
					CHECK(result.get_h1_amplitude() == Catch::Approx(0.0f).margin(0.01f));
				}
			}
		SECTION("Finds true fundamental even when upper harmonics dominate")
		{
			HarmonicPitchSettings config;
			config.min_amplitude = 0.05f;
			config.allow_single_peak_mode = false;

			const int num_bands = 128;
			AudioBuffer128 centers(num_bands);
			AudioBuffer128 envelope(num_bands);
			const float fmin = 100.0f;
			const float fmax = 4000.0f;
			for (int i = 0; i < num_bands; ++i)
			{
				centers[i] = fmin + (fmax - fmin) * (float(i) / float(num_bands - 1));
				envelope[i] = 0.0f;
			}

			const float fundamental = 220.0f;
			stamp_gaussian_peak(envelope, centers, fundamental, 0.2f, 30.0f);
			stamp_gaussian_peak(envelope, centers, fundamental * 2.0f, 1.0f, 25.0f);
			stamp_gaussian_peak(envelope, centers, fundamental * 3.0f, 0.7f, 25.0f);

			HarmonicPitchResult result{};
			REQUIRE(HarmonicPitch::find_harmonic_features(config, centers, envelope, result));
			CHECK(result.h1_f0_hz == Catch::Approx(fundamental).margin(5.0f));
			CHECK(result.get_h1_amplitude() < 0.4f);
		}

		SECTION("Continuation rejects weak energy or large jumps")
		{
			const int num_bands = 128;
			AudioBuffer128 centers(num_bands);
			AudioBuffer128 envelope(num_bands);
			const float fmin = 80.0f;
			const float fmax = 3000.0f;
			for (int i = 0; i < num_bands; ++i)
			{
				centers[i] = fmin + (fmax - fmin) * (float(i) / float(num_bands - 1));
				envelope[i] = 0.0f;
			}

			HarmonicPitchSettings config;
			config.min_amplitude = 0.05f;
			config.min_total_continuation_amplitude = 0.8f;

			HarmonicPitchResult prev{};
			prev.h1_f0_hz = 250.0f;
			prev.harmonic_amplitudes.add(0.6f);

			stamp_gaussian_peak(envelope, centers, prev.h1_f0_hz, 0.2f, 20.0f);
			HarmonicPitchResult continued{};
			CHECK_FALSE(HarmonicPitch::try_continue_previous_result(config, centers, envelope, prev, continued));

			for (size_t i = 0; i < envelope.size(); ++i)
				envelope[i] = 0.0f;
			config.min_total_continuation_amplitude = 0.2f;
			stamp_gaussian_peak(envelope, centers, prev.h1_f0_hz * 2.0f, 1.0f, 15.0f);
			CHECK_FALSE(HarmonicPitch::try_continue_previous_result(config, centers, envelope, prev, continued));
		}

		SECTION("Continuation bridges short gaps by probing nearby bands")
		{
			const int num_bands = 128;
			AudioBuffer128 centers(num_bands);
			AudioBuffer128 envelope(num_bands);
			const float fmin = 80.0f;
			const float fmax = 1500.0f;
			for (int i = 0; i < num_bands; ++i)
			{
				centers[i] = fmin + (fmax - fmin) * (float(i) / float(num_bands - 1));
				envelope[i] = 0.0f;
			}

			HarmonicPitchSettings config;
			config.min_amplitude = 0.05f;
			config.min_total_continuation_amplitude = 0.3f;
			config.continuation_search_radius = 3;

			HarmonicPitchResult prev{};
			prev.h1_f0_hz = 400.0f;
			prev.harmonic_amplitudes.add(0.5f);

				stamp_gaussian_peak(envelope, centers, prev.h1_f0_hz, 0.6f, 25.0f);
				stamp_gaussian_peak(envelope, centers, prev.h1_f0_hz * 2.0f, 0.3f, 25.0f);
			const int prev_band = find_closest_frequency_center_index(centers, prev.h1_f0_hz);
			REQUIRE(prev_band >= 0);
			envelope[prev_band] = 0.0f; // simulate a single-bin dropout

			HarmonicPitchResult continued{};
			REQUIRE(HarmonicPitch::try_continue_previous_result(config, centers, envelope, prev, continued));
			CHECK(continued.h1_f0_hz == Catch::Approx(prev.h1_f0_hz).margin(8.0f));
		}

	}
}

}; // namespace robotick::test
