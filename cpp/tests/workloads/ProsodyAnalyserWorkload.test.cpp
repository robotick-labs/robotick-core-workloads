// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#include "robotick/framework/containers/FixedVector.h"
#include "robotick/systems/audio/AudioFrame.h"
#include "robotick/systems/auditory/CochlearFrame.h"
#include "robotick/systems/auditory/HarmonicPitch.h"
#include "robotick/systems/auditory/ProsodyMath.h"
#include "robotick/systems/auditory/ProsodyState.h"

#include <catch2/catch_all.hpp>
#include <cmath>

namespace robotick::test
{
	namespace
	{
		struct ProsodyPipelineHarness
		{
			struct Config
			{
				float harmonic_floor_db = -60.0f;
				float speaking_rate_decay = 0.95f;
				float pitch_smooth_alpha = 0.2f;
				float rms_smooth_alpha = 0.2f;
				float voiced_falloff_rate_hz = 5.0f;
				float min_pitch_hz = 60.0f;
				float max_pitch_hz = 600.0f;
			} config;

			float previous_pitch_hz = 0.0f;
			float smoothed_pitch_hz = 0.0f;
			float smoothed_rms = 0.0f;
			RelativeVariationTracker pitch_tracker;
			RelativeVariationTracker rms_tracker;
			SpeakingRateTracker speaking_tracker;

			ProsodyState tick(const AudioFrame& frame, const HarmonicPitchResult& pitch, float time_now, float delta_time)
			{
				ProsodyState prosody{};

				double energy = 0.0;
				for (float sample : frame.samples)
				{
					energy += static_cast<double>(sample) * static_cast<double>(sample);
				}
				const float frame_energy = robotick::max(static_cast<float>(energy), 1e-12f);
				const float rms = frame.samples.empty() ? 0.0f : static_cast<float>(sqrt(energy / static_cast<double>(frame.samples.size())));
				smoothed_rms = apply_exponential_smoothing(smoothed_rms, rms, config.rms_smooth_alpha);
				prosody.rms = smoothed_rms;

				const bool voiced_now = (pitch.h1_f0_hz >= config.min_pitch_hz && pitch.h1_f0_hz <= config.max_pitch_hz);
				prosody.voiced_confidence =
					update_voiced_confidence(voiced_now, prosody.voiced_confidence, delta_time, config.voiced_falloff_rate_hz);

				if (!voiced_now)
				{
					previous_pitch_hz = 0.0f;
					speaking_tracker.was_voiced = false;
					decay_speaking_rate_tracker(speaking_tracker, config.speaking_rate_decay);
					prosody.is_voiced = false;
					return prosody;
				}

				prosody.is_voiced = true;
				prosody.voiced_confidence = 1.0f;

				smoothed_pitch_hz = apply_exponential_smoothing(smoothed_pitch_hz, pitch.h1_f0_hz, config.pitch_smooth_alpha);
				prosody.pitch_hz = smoothed_pitch_hz;

				if (previous_pitch_hz > 0.0f)
				{
					const float slope = (smoothed_pitch_hz - previous_pitch_hz) / delta_time;
					prosody.pitch_slope_hz_per_s = slope;
				}
				previous_pitch_hz = smoothed_pitch_hz;

				float harmonic_energy = 0.0f;
				for (size_t i = 0; i < pitch.harmonic_amplitudes.size(); ++i)
				{
					const float amp = pitch.harmonic_amplitudes[i];
					harmonic_energy += amp * amp;
				}
				prosody.harmonicity_hnr_db = compute_harmonicity_hnr_db(frame_energy, harmonic_energy, config.harmonic_floor_db);
				prosody.spectral_brightness = compute_spectral_brightness(pitch);
				const FormantRatios ratios = compute_formant_ratios(pitch, static_cast<float>(frame.sample_rate));
				prosody.formant1_ratio = ratios.first;
				prosody.formant2_ratio = ratios.second;
				const HarmonicDescriptors desc = compute_harmonic_descriptors(pitch, static_cast<float>(frame.sample_rate));
				prosody.h1_to_h2_db = desc.h1_to_h2_db;
				prosody.harmonic_tilt_db_per_h = desc.harmonic_tilt_db_per_h;
				prosody.even_odd_ratio = desc.even_odd_ratio;
				prosody.harmonic_support_ratio = desc.harmonic_support_ratio;
				prosody.centroid_ratio = desc.centroid_ratio;

				prosody.jitter = update_relative_variation(pitch_tracker, pitch.h1_f0_hz);
				prosody.shimmer = update_relative_variation(rms_tracker, rms);
				prosody.speaking_rate_sps = update_speaking_rate_on_voiced(speaking_tracker, time_now, config.speaking_rate_decay);

				return prosody;
			}
		};

		inline void fill_band_centers(AudioBuffer128& centers, float min_hz, float max_hz)
		{
			const size_t count = centers.capacity();
			const float step = (max_hz - min_hz) / static_cast<float>(count);
			for (size_t i = 0; i < count; ++i)
			{
				centers.add(min_hz + step * static_cast<float>(i));
			}
		}

		inline void synthesize_envelope(CochlearFrame& frame, float f0_hz, float brightness_scale)
		{
			frame.envelope.clear();
			frame.band_center_hz.clear();
			fill_band_centers(frame.band_center_hz, 80.0f, 8000.0f);
			for (size_t i = 0; i < frame.band_center_hz.size(); ++i)
			{
				frame.envelope.add(0.001f);
			}

			const int harmonics = 5;
			for (int h = 1; h <= harmonics; ++h)
			{
				const float harmonic_freq = f0_hz * static_cast<float>(h);
				float best_diff = 1e9f;
				size_t best_idx = 0;
				for (size_t band = 0; band < frame.band_center_hz.size(); ++band)
				{
					const float diff = fabsf(frame.band_center_hz[band] - harmonic_freq);
					if (diff < best_diff)
					{
						best_diff = diff;
						best_idx = band;
					}
				}
				float amplitude = 0.8f / static_cast<float>(h);
				if (h >= 4)
				{
					amplitude *= brightness_scale;
				}
				frame.envelope[best_idx] = amplitude;
			}
		}

		inline void synthesize_audio(AudioFrame& frame, float frequency_hz, float duration_s)
		{
			frame.samples.clear();
			frame.sample_rate = 16000;
			const int total_samples = static_cast<int>(frame.sample_rate * duration_s);
			const float dt = 1.0f / static_cast<float>(frame.sample_rate);
			for (int i = 0; i < total_samples && i < static_cast<int>(frame.samples.capacity()); ++i)
			{
				const float t = static_cast<float>(i) * dt;
				const float value = sinf(2.0f * static_cast<float>(M_PI) * frequency_hz * t);
				frame.samples.add(value);
			}
		}
	} // namespace
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

	TEST_CASE("Unit/Workloads/ProsodyAnalyser/HarmonicDescriptors")
	{
		const float sample_rate = 16000.0f;

		SECTION("H1-to-H2 captures the dB gap between first two harmonics")
		{
			HarmonicPitchResult hp;
			hp.h1_f0_hz = 100.0f;
			hp.harmonic_amplitudes.add(1.0f);
			hp.harmonic_amplitudes.add(0.5f);

			const HarmonicDescriptors descriptors = compute_harmonic_descriptors(hp, sample_rate);

			CHECK(descriptors.h1_to_h2_db == Catch::Approx(6.02f).margin(0.05f));
		}

		SECTION("Harmonic tilt follows the slope of the harmonic envelope")
		{
			HarmonicPitchResult hp;
			hp.h1_f0_hz = 120.0f;
			hp.harmonic_amplitudes.add(1.0f);
			hp.harmonic_amplitudes.add(0.5f);
			hp.harmonic_amplitudes.add(0.25f);

			const HarmonicDescriptors descriptors = compute_harmonic_descriptors(hp, sample_rate);

			CHECK(descriptors.harmonic_tilt_db_per_h == Catch::Approx(-6.0f).margin(0.2f));
		}

		SECTION("Even/odd ratio grows when even harmonics dominate")
		{
			HarmonicPitchResult hp;
			hp.h1_f0_hz = 150.0f;
			hp.harmonic_amplitudes.add(1.0f);
			hp.harmonic_amplitudes.add(0.8f);
			hp.harmonic_amplitudes.add(0.2f);
			hp.harmonic_amplitudes.add(0.8f);

			const HarmonicDescriptors descriptors = compute_harmonic_descriptors(hp, sample_rate);

			CHECK(descriptors.even_odd_ratio == Catch::Approx((0.8f + 0.8f) / (1.0f + 0.2f)).margin(1e-3f));
		}

		SECTION("Support ratio counts harmonics above the -12 dB threshold")
		{
			HarmonicPitchResult hp;
			hp.h1_f0_hz = 160.0f;
			hp.harmonic_amplitudes.add(1.0f);
			hp.harmonic_amplitudes.add(0.4f);
			hp.harmonic_amplitudes.add(0.2f);
			hp.harmonic_amplitudes.add(0.26f);

			const HarmonicDescriptors descriptors = compute_harmonic_descriptors(hp, sample_rate);

			CHECK(descriptors.harmonic_support_ratio == Catch::Approx(0.75f).margin(1e-3f));
		}

		SECTION("Centroid ratio normalizes the weighted harmonic index")
		{
			HarmonicPitchResult hp;
			hp.h1_f0_hz = 200.0f;
			hp.harmonic_amplitudes.add(1.0f);
			hp.harmonic_amplitudes.add(1.0f);
			hp.harmonic_amplitudes.add(1.0f);
			hp.harmonic_amplitudes.add(1.0f);

			const HarmonicDescriptors descriptors = compute_harmonic_descriptors(hp, sample_rate);

			CHECK(descriptors.centroid_ratio == Catch::Approx(0.625f).margin(1e-3f));
		}

		SECTION("Formants report normalized frequencies for the two strongest peaks")
		{
			HarmonicPitchResult hp;
			hp.h1_f0_hz = 100.0f;
			hp.harmonic_amplitudes.add(0.01f);
			hp.harmonic_amplitudes.add(2.0f); // dominant second harmonic (~200 Hz)
			hp.harmonic_amplitudes.add(0.05f);
			hp.harmonic_amplitudes.add(0.001f);
			hp.harmonic_amplitudes.add(0.0001f);
			hp.harmonic_amplitudes.add(1.5f); // dominant sixth harmonic (~600 Hz)
			hp.harmonic_amplitudes.add(0.0001f);

			const HarmonicDescriptors descriptors = compute_harmonic_descriptors(hp, sample_rate);

			CHECK(descriptors.formant1_ratio == Catch::Approx(200.0f / 8000.0f).margin(0.005f));
			CHECK(descriptors.formant2_ratio == Catch::Approx(500.0f / 8000.0f).margin(0.005f));
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

	TEST_CASE("Unit/Workloads/ProsodyAnalyser/EMASmoothing")
	{
		const auto step_response = [](float initial, float input, float alpha, int steps)
		{
			float value = initial;
			for (int i = 0; i < steps; ++i)
			{
				value = apply_exponential_smoothing(value, input, alpha);
			}
			return value;
		};

		SECTION("RMS smoothing matches analytical EMA step response")
		{
			const float alpha = 0.2f;
			const float initial = 0.0f;
			const float target = 1.0f;
			const int steps = 5;

			const float smoothed = step_response(initial, target, alpha, steps);
			const float expected = target - (target - initial) * powf(1.0f - alpha, static_cast<float>(steps));

			CHECK(smoothed == Catch::Approx(expected).margin(1e-4f));
		}

		SECTION("Pitch smoothing follows the same EMA formula")
		{
			const float alpha = 0.1f;
			const float initial = 120.0f;
			const float target = 240.0f;
			const int steps = 8;

			const float smoothed = step_response(initial, target, alpha, steps);
			const float expected = target - (target - initial) * powf(1.0f - alpha, static_cast<float>(steps));

			CHECK(smoothed == Catch::Approx(expected).margin(1e-3f));
		}
	}

	TEST_CASE("Unit/Workloads/ProsodyAnalyser/SpeakingTimeline")
	{
		SECTION("Voiced confidence decays linearly during silence")
		{
			const float falloff_rate = 1.0f;
			const float delta_time = 0.1f;

			float confidence = update_voiced_confidence(true, 0.0f, delta_time, falloff_rate);
			REQUIRE(confidence == Catch::Approx(1.0f));

			for (int i = 0; i < 5; ++i)
			{
				confidence = update_voiced_confidence(false, confidence, delta_time, falloff_rate);
			}

			CHECK(confidence == Catch::Approx(0.5f).margin(1e-3f));
		}

		SECTION("Speaking-rate tracker matches reference timeline model")
		{
			SpeakingRateTracker tracker;
			const float decay = 0.95f;
			float current_time = 0.0f;
			float latest_rate = 0.0f;

			float reference_rate = 0.0f;
			float reference_last_onset_time = 0.0f;
			bool reference_was_voiced = false;

			const auto reference_voiced = [&](float time_now)
			{
				if (!reference_was_voiced)
				{
					const float gap_seconds = robotick::max(0.0f, time_now - reference_last_onset_time);
					const float instant_rate = (gap_seconds > 0.05f) ? (1.0f / gap_seconds) : 0.0f;
					reference_rate = update_speaking_rate_sps(reference_rate, instant_rate, decay, gap_seconds);
					reference_last_onset_time = time_now;
				}
				reference_was_voiced = true;
			};

			const auto reference_silence = [&]()
			{
				reference_rate *= decay;
				reference_was_voiced = false;
			};

			const auto simulate_voiced_segment = [&](float duration)
			{
				latest_rate = update_speaking_rate_on_voiced(tracker, current_time, decay);
				reference_voiced(current_time);
				current_time += duration;
			};

			const auto simulate_silence = [&](float duration, int steps)
			{
				const float step = duration / static_cast<float>(steps);
				for (int i = 0; i < steps; ++i)
				{
					decay_speaking_rate_tracker(tracker, decay);
					reference_silence();
					current_time += step;
				}
			};

			// Initial voiced burst with no prior silence.
			simulate_voiced_segment(0.5f);

			// Repeat [0.5 s silence, 0.5 s voiced] to build cadence.
			for (int i = 0; i < 4; ++i)
			{
				simulate_silence(0.5f, 10);
				simulate_voiced_segment(0.5f);
			}

			CHECK(latest_rate == Catch::Approx(reference_rate).margin(1e-4f));

			// Long pause (3 s) followed by another onset should still align with the reference model.
			simulate_silence(3.0f, 30);
			simulate_voiced_segment(0.5f);

			CHECK(latest_rate == Catch::Approx(reference_rate).margin(1e-4f));
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

	TEST_CASE("Integration/Auditory/HarmonicPitchToProsody")
	{
		HarmonicPitchSettings pitch_settings;
		pitch_settings.min_amplitude = 0.01f;
		pitch_settings.min_peak_falloff_norm = 0.05f;
		pitch_settings.allow_single_peak_mode = true;

		ProsodyPipelineHarness harness;
		HarmonicPitchResult prev{};

		float time_now = 0.0f;
		const float delta_time = 0.05f;

		FixedVector<float, 32> detected_pitches;
		FixedVector<float, 32> brightness_values;
		FixedVector<float, 32> confidence_values;

		for (int frame_idx = 0; frame_idx < 6; ++frame_idx)
		{
			const float base_pitch = 140.0f + frame_idx * 10.0f;
			const float brightness_scale = (frame_idx >= 3) ? 1.5f : 1.0f;

			CochlearFrame cochlear{};
			synthesize_envelope(cochlear, base_pitch, brightness_scale);
			AudioFrame audio{};
			synthesize_audio(audio, base_pitch, delta_time);
			cochlear.timestamp = time_now;

			HarmonicPitchResult current{};
			const bool ok =
				HarmonicPitch::find_or_continue_harmonic_features(pitch_settings, cochlear.band_center_hz, cochlear.envelope, prev, current);
			REQUIRE(ok);
			prev = current;

			const ProsodyState prosody = harness.tick(audio, current, time_now, delta_time);
			detected_pitches.add(prosody.pitch_hz);
			brightness_values.add(prosody.spectral_brightness);
			confidence_values.add(prosody.voiced_confidence);

			time_now += delta_time;
		}

		for (int silent = 0; silent < 3; ++silent)
		{
			AudioFrame silent_audio{};
			CochlearFrame silent_cochlear{};
			synthesize_envelope(silent_cochlear, 0.0f, 1.0f);
			silent_cochlear.timestamp = time_now;
			HarmonicPitchResult empty{};
			prev = {};
			const ProsodyState prosody = harness.tick(silent_audio, empty, time_now, delta_time);
			confidence_values.add(prosody.voiced_confidence);
			time_now += delta_time;
		}

		REQUIRE(detected_pitches.size() >= 3);
		float last_nonzero = 0.0f;
		for (size_t i = 0; i < detected_pitches.size(); ++i)
		{
			const float value = detected_pitches[i];
			if (value > 0.0f)
			{
				if (last_nonzero > 0.0f)
				{
					CHECK(value >= last_nonzero - 5.0f);
				}
				last_nonzero = value;
			}
		}

		CHECK(brightness_values[4] > brightness_values[1]);
		REQUIRE(confidence_values.size() > 0);
		CHECK(confidence_values[confidence_values.size() - 1] < 0.2f);
	}
} // namespace robotick::test
