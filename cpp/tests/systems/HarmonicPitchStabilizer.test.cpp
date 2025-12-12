// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#include "robotick/systems/auditory/HarmonicPitchStabilizer.h"

#include <catch2/catch_all.hpp>

namespace robotick::test
{
	using Catch::Approx;

	namespace
	{
		HarmonicPitchResult make_result(float f0_hz, float h1_amp = 1.0f)
		{
			HarmonicPitchResult result{};
			result.h1_f0_hz = f0_hz;
			result.harmonic_amplitudes.add(h1_amp);
			result.harmonic_amplitudes.add(h1_amp * 0.5f);
			return result;
		}
	} // namespace

	TEST_CASE("Unit/Systems/Auditory/HarmonicPitchStabilizer/WarmupWindow")
	{
		HarmonicPitchStabilizer stabilizer;
		HarmonicPitchStabilizerConfig config{};
		config.warmup_frame_count = 3;
		config.max_hold_frames = 2;

		stabilizer.configure(config);
		HarmonicPitchResult out{};

		REQUIRE(stabilizer.process_valid_frame(make_result(100.0f), out));
		REQUIRE(out.h1_f0_hz == Approx(100.0f));

		REQUIRE(stabilizer.process_valid_frame(make_result(102.0f), out));
		REQUIRE(out.h1_f0_hz == Approx((100.0f + 102.0f) / 2.0f));

		REQUIRE(stabilizer.process_valid_frame(make_result(104.0f), out));
		REQUIRE(out.h1_f0_hz == Approx((100.0f + 102.0f + 104.0f) / 3.0f).margin(0.01f));

		REQUIRE(stabilizer.process_valid_frame(make_result(110.0f), out));
		REQUIRE(out.h1_f0_hz == Approx(110.0f));
	}

	TEST_CASE("Unit/Systems/Auditory/HarmonicPitchStabilizer/HoldAndReset")
	{
		HarmonicPitchStabilizer stabilizer;
		HarmonicPitchStabilizerConfig config{};
		config.warmup_frame_count = 2;
		config.max_hold_frames = 1;
		stabilizer.configure(config);

		HarmonicPitchResult out{};
		REQUIRE(stabilizer.process_valid_frame(make_result(90.0f), out));
		REQUIRE(out.h1_f0_hz == Approx(90.0f));
		REQUIRE(stabilizer.process_valid_frame(make_result(92.0f), out));
		REQUIRE(out.h1_f0_hz == Approx((90.0f + 92.0f) / 2.0f));

		REQUIRE(stabilizer.process_missing_frame(out));
		REQUIRE(out.h1_f0_hz == Approx((90.0f + 92.0f) / 2.0f));

		REQUIRE_FALSE(stabilizer.process_missing_frame(out));

		REQUIRE(stabilizer.process_valid_frame(make_result(120.0f), out));
		REQUIRE(out.h1_f0_hz == Approx(120.0f));
		REQUIRE(stabilizer.process_valid_frame(make_result(122.0f), out));
		REQUIRE(out.h1_f0_hz == Approx((120.0f + 122.0f) / 2.0f));
	}

} // namespace robotick::test
