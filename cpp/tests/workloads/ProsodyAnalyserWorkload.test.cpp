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

} // namespace robotick::test
