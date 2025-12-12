// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#include "robotick/systems/auditory/SnakePitchTracker.h"
#include "robotick/framework/containers/FixedVector.h"
#include "robotick/framework/math/MathUtils.h"

#include <catch2/catch_all.hpp>
#include <cmath>

namespace robotick::test
{
	namespace
	{
		constexpr size_t kBandCount = 64;

		struct PeakSpec
		{
			float freq = 0.0f;
			float amplitude = 0.0f;
		};

		using PeakList = FixedVector<PeakSpec, 32>;

		inline CochlearFrame make_frame(const PeakList& peaks)
		{
			CochlearFrame frame{};
			frame.envelope.clear();
			frame.band_center_hz.clear();

			const float min_hz = 80.0f;
			const float max_hz = 4000.0f;
			for (size_t i = 0; i < kBandCount; ++i)
			{
				const float t = static_cast<float>(i) / static_cast<float>(kBandCount - 1);
				const float freq = min_hz * powf(max_hz / min_hz, t);
				frame.band_center_hz.add(freq);
				frame.envelope.add(0.0001f);
			}

			for (size_t peak_index = 0; peak_index < peaks.size(); ++peak_index)
			{
				const PeakSpec& peak = peaks[peak_index];
				float best_diff = 1e9f;
				size_t best_idx = 0;
				for (size_t i = 0; i < frame.band_center_hz.size(); ++i)
				{
					const float diff = fabsf(frame.band_center_hz[i] - peak.freq);
					if (diff < best_diff)
					{
						best_diff = diff;
						best_idx = i;
					}
				}

				frame.envelope[best_idx] = robotick::max(frame.envelope[best_idx], peak.amplitude);
				if (best_idx + 1 < frame.envelope.size())
				{
					frame.envelope[best_idx + 1] = robotick::max(frame.envelope[best_idx + 1], peak.amplitude * 0.5f);
				}
				if (best_idx > 0)
				{
					frame.envelope[best_idx - 1] = robotick::max(frame.envelope[best_idx - 1], peak.amplitude * 0.5f);
				}
			}

			return frame;
		}

		inline CochlearFrame make_harmonic_frame(float fundamental_hz, float amplitude_scale = 1.0f)
		{
			PeakList peaks;
			for (int h = 1; h <= 5; ++h)
			{
				const float freq = fundamental_hz * static_cast<float>(h);
				const float amp = amplitude_scale * (0.8f / static_cast<float>(h));
				if (!peaks.full())
				{
					peaks.add(PeakSpec{freq, amp});
				}
			}
			return make_frame(peaks);
		}

		inline CochlearFrame make_silent_frame()
		{
			PeakList empty;
			return make_frame(empty);
		}
	} // namespace

	TEST_CASE("Unit/Systems/SnakePitchTracker/StableFundamental")
	{
		SnakePitchTracker tracker;
		SnakePitchTrackerConfig config{};
		tracker.configure(config);

		HarmonicPitchResult result{};

		for (int i = 0; i < 5; ++i)
		{
			const CochlearFrame frame = make_harmonic_frame(220.0f);
			const bool has_pitch = tracker.update(frame, result);
			REQUIRE(has_pitch);
			CHECK(result.h1_f0_hz == Catch::Approx(220.0f).margin(5.0f));
			CHECK(result.harmonic_amplitudes[0] > result.harmonic_amplitudes[1]);
		}
	}

	TEST_CASE("Unit/Systems/SnakePitchTracker/DropoutTolerance")
	{
		SnakePitchTracker tracker;
		SnakePitchTrackerConfig config{};
		tracker.configure(config);

		HarmonicPitchResult result{};

		// Prime tracker with voiced frames.
		for (int i = 0; i < 3; ++i)
		{
			const CochlearFrame voiced = make_harmonic_frame(180.0f);
			REQUIRE(tracker.update(voiced, result));
		}

		// Two silent frames should not kill the ridge (keep-alive default is 4 frames).
		for (int i = 0; i < 2; ++i)
		{
			const CochlearFrame silent = make_silent_frame();
			const bool has_pitch = tracker.update(silent, result);
			CHECK(has_pitch);
			CHECK(result.h1_f0_hz == Catch::Approx(180.0f).margin(5.0f));
		}
	}

	TEST_CASE("Unit/Systems/SnakePitchTracker/HarmonicGrouping")
	{
		SnakePitchTracker tracker;
		SnakePitchTrackerConfig config{};
		tracker.configure(config);

		HarmonicPitchResult result{};

		// Create two simultaneous ridges: 200 Hz voice + 320 Hz distractor.
		PeakList peaks;
		peaks.add(PeakSpec{200.0f, 0.9f});
		peaks.add(PeakSpec{400.0f, 0.6f});
		peaks.add(PeakSpec{600.0f, 0.4f});
		peaks.add(PeakSpec{320.0f, 0.7f});
		peaks.add(PeakSpec{640.0f, 0.6f});

		const CochlearFrame frame = make_frame(peaks);

		REQUIRE(tracker.update(frame, result));
		CHECK(result.h1_f0_hz == Catch::Approx(200.0f).margin(5.0f));
		CHECK(result.harmonic_amplitudes[0] > result.harmonic_amplitudes[1]);
		CHECK(result.harmonic_amplitudes[1] > 0.0f);
	}
} // namespace robotick::test
