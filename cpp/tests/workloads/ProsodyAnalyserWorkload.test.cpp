// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#include "robotick/systems/auditory/HarmonicPitch.h"
#include "robotick/systems/auditory/ProsodyMath.h"

#include <catch2/catch_all.hpp>

namespace robotick::test
{
	TEST_CASE("Unit/Workloads/ProsodyAnalyser/HarmonicityHNR")
	{
		SECTION("Balanced harmonic and noise energy yields 0 dB")
		{
			const float frame_energy = 2.0f;	// total power
			const float harmonic_energy = 1.0f; // half harmonic, half noise
			const float expected_db = 0.0f;

			const float hnr = compute_harmonicity_hnr_db(frame_energy, harmonic_energy, -60.0f);

			CHECK(hnr == Catch::Approx(expected_db).margin(0.1f));
		}

		SECTION("Noise-dominated frame respects floor")
		{
			const float frame_energy = 1.0f;
			const float harmonic_energy = 1e-9f;
			const float floor_db = -40.0f;

			const float hnr = compute_harmonicity_hnr_db(frame_energy, harmonic_energy, floor_db);

			CHECK(hnr == Catch::Approx(floor_db).margin(0.1f));
		}
	}

	TEST_CASE("Unit/Workloads/ProsodyAnalyser/FormantNormalization")
	{
		HarmonicPitchResult low_f0;
		low_f0.h1_f0_hz = 100.0f;
		low_f0.harmonic_amplitudes.add(0.1f);
		low_f0.harmonic_amplitudes.add(0.2f);
		low_f0.harmonic_amplitudes.add(0.3f);
		low_f0.harmonic_amplitudes.add(0.9f); // strong fourth harmonic (~400 Hz)
		low_f0.harmonic_amplitudes.add(0.2f);

		HarmonicPitchResult high_f0;
		high_f0.h1_f0_hz = 250.0f;
		high_f0.harmonic_amplitudes.add(0.1f);
		high_f0.harmonic_amplitudes.add(0.9f); // second harmonic (~500 Hz)
		high_f0.harmonic_amplitudes.add(0.2f);
		high_f0.harmonic_amplitudes.add(0.1f);

		const float sample_rate = 16000.0f;

		const FormantRatios low_ratios = compute_formant_ratios(low_f0, sample_rate);
		const FormantRatios high_ratios = compute_formant_ratios(high_f0, sample_rate);

		CHECK(low_ratios.first == Catch::Approx(high_ratios.first).margin(0.05f));
	}

} // namespace robotick::test
