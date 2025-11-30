// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#include "robotick/systems/audio/AudioSystem.h"

#include <catch2/catch_test_macros.hpp>

namespace robotick::tests
{
	TEST_CASE("AudioSystem updates drop stats when helper invoked", "[audio]")
	{
		AudioSystem::reset_backpressure_stats();
		AudioSystem::set_output_spec_for_test(44100, 2);

		// Simulate a 10ms drop worth of stereo samples
		const uint32_t frames = 441; // ~10ms at 44.1kHz
		const uint32_t bytes = frames * 2 * sizeof(float);
		AudioSystem::record_drop_for_test(bytes);

		const auto stats = AudioSystem::get_backpressure_stats();
		REQUIRE(stats.drop_events == 1);
		REQUIRE(stats.dropped_ms > 0.0f);
	}
} // namespace robotick::tests
