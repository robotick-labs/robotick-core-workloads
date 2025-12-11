// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#include "robotick/systems/auditory/HarmonicPitch.h"
#include "robotick/systems/auditory/ProsodyMath.h"

#include <catch2/catch_all.hpp>

namespace robotick::test
{
	TEST_CASE("Unit/Workloads/ProsodyAnalyser/HarmonicityHNR")
	{
		// Balanced spectrum: harmonic energy matches noise energy → 0 dB
		SECTION("Balanced harmonic and noise energy yields 0 dB")
		{
			const float frame_energy = 2.0f;	// total power
			const float harmonic_energy = 1.0f; // half harmonic, half noise
			const float expected_db = 0.0f;

			const float hnr = compute_harmonicity_hnr_db(frame_energy, harmonic_energy, -60.0f);

			CHECK(hnr == Catch::Approx(expected_db).margin(0.1f));
		}

		// Noise-dominated spectrum should clamp to the configured floor
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
		SECTION("Normalized formant ratios stay stable when F0 changes")
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
	}

	TEST_CASE("Unit/Workloads/ProsodyAnalyser/RelativeVariation")
	{
		SECTION("Alternating pitch exhibits expected jitter ratio")
		{
			RelativeVariationTracker tracker;

			CHECK(update_relative_variation(tracker, 200.0f) == Catch::Approx(0.0f));
			CHECK(update_relative_variation(tracker, 400.0f) == Catch::Approx(1.0f).margin(1e-3f));
			CHECK(update_relative_variation(tracker, 200.0f) == Catch::Approx(0.5f).margin(1e-3f));
		}

		SECTION("Silence resets shimmer tracker for fresh measurements")
		{
			RelativeVariationTracker tracker;
			CHECK(update_relative_variation(tracker, 1.0f) == Catch::Approx(0.0f));
			CHECK(update_relative_variation(tracker, 0.0f) == Catch::Approx(0.0f));
			CHECK(update_relative_variation(tracker, 1.0f) == Catch::Approx(0.0f));
		}
	}

	TEST_CASE("Unit/Workloads/ProsodyAnalyser/SpectralBrightness")
	{
		SECTION("Flat spectrum reports near-zero slope")
		{
			HarmonicPitchResult flat;
			flat.h1_f0_hz = 200.0f;
			flat.harmonic_amplitudes.add(1.0f);
			flat.harmonic_amplitudes.add(1.0f);
			flat.harmonic_amplitudes.add(1.0f);
			flat.harmonic_amplitudes.add(1.0f);

			CHECK(compute_spectral_brightness(flat) == Catch::Approx(0.0f).margin(1e-3f));
		}

		SECTION("Treble-heavy spectrum yields positive slope")
		{
			HarmonicPitchResult bright;
			bright.h1_f0_hz = 200.0f;
			bright.harmonic_amplitudes.add(0.2f);
			bright.harmonic_amplitudes.add(0.4f);
			bright.harmonic_amplitudes.add(0.8f);
			bright.harmonic_amplitudes.add(1.6f);

			CHECK(compute_spectral_brightness(bright) > 0.1f);
		}
	}

	TEST_CASE("Unit/Workloads/ProsodyAnalyser/VoicedConfidenceDecay")
	{
		const float falloff_rate = 1.0f;
		const float delta_time = 0.1f;

		SECTION("Voiced frame forces confidence to 1")
		{
			const float voiced_conf = update_voiced_confidence(true, 0.0f, delta_time, falloff_rate);
			CHECK(voiced_conf == Catch::Approx(1.0f));
		}

		SECTION("Silent frames decay confidence linearly")
		{
			const float voiced_conf = update_voiced_confidence(true, 0.0f, delta_time, falloff_rate);
			const float silent_conf = update_voiced_confidence(false, voiced_conf, delta_time, falloff_rate);
			CHECK(silent_conf == Catch::Approx(0.9f).margin(1e-3f));
		}
	}

	TEST_CASE("Unit/Workloads/ProsodyAnalyser/SpeakingRate")
	{
		SECTION("Short pauses push tracker toward instant rate")
		{
			float tracker = 0.0f;
			const float decay = 0.8f;
			tracker = update_speaking_rate_sps(tracker, 2.0f, decay, 0.5f);
			CHECK(tracker == Catch::Approx(0.4f).margin(0.01f));
		}

		SECTION("Long pauses never drive tracker to zero")
		{
			float tracker = 0.2f;
			const float decay = 0.8f;
			tracker = update_speaking_rate_sps(tracker, 0.0f, decay, 3.0f);
			CHECK(tracker > 0.2f);
		}
	}
} // namespace robotick::test
