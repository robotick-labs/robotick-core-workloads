// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#include "robotick/systems/auditory/ProsodyFusionMath.h"

#include <catch2/catch_all.hpp>

namespace robotick::test
{
	TEST_CASE("Unit/Systems/Auditory/ProsodyFusionMath/LinkEvaluation")
	{
		ProsodyLinkConstraints constraints;
		constraints.max_jump_hz = 80.0f;

		SECTION("Smooth transitions stay connected")
		{
			ProsodyLinkSample prev{};
			prev.pitch_hz = 200.0f;
			prev.rms = 0.3f;
			prev.time_sec = 0.0f;

			ProsodyLinkSample curr{};
			curr.pitch_hz = 210.0f;
			curr.rms = 0.4f;
			curr.time_sec = 0.05f;

			const ProsodyLinkEvaluation eval = evaluate_prosody_link(constraints, prev, curr);
			CHECK(eval.connect);
			CHECK(eval.link_rms == Catch::Approx(0.35f));
		}

		SECTION("Large jumps break the contour")
		{
			ProsodyLinkSample prev{};
			prev.pitch_hz = 200.0f;
			prev.rms = 0.3f;
			prev.time_sec = 0.0f;

			ProsodyLinkSample curr{};
			curr.pitch_hz = 400.0f; // jump exceeds limit
			curr.rms = 0.4f;
			curr.time_sec = 0.05f;

			const ProsodyLinkEvaluation eval = evaluate_prosody_link(constraints, prev, curr);
			CHECK_FALSE(eval.connect);
			CHECK(eval.link_rms == Catch::Approx(0.0f));
		}
	}
} // namespace robotick::test
