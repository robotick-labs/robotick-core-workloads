// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#include "robotick/systems/MuJoCoPhysics.h"

#include <catch2/catch_test_macros.hpp>

namespace robotick::tests
{
#if defined(ROBOTICK_PLATFORM_DESKTOP) || defined(ROBOTICK_PLATFORM_LINUX)
	namespace
	{
		// Minimal MuJoCo scene with empty worldbody, used for smoke tests (no visuals required).
		constexpr char kMinimalModelPath[] = ROBOTICK_CORE_ROOT "/cpp/tests/data/mujoco/minimal.xml";
	} // namespace

	TEST_CASE("Unit/Systems/MuJoCoPhysics")
	{
		SECTION("Load invalid path returns false")
		{
			MuJoCoPhysics physics;
			REQUIRE_FALSE(physics.load_from_xml("does_not_exist.xml"));
		}

		SECTION("Loads model, steps sim, and allocates/copies/frees render snapshot")
		{
			MuJoCoPhysics physics;
			REQUIRE(physics.load_from_xml(kMinimalModelPath));
			REQUIRE(physics.is_loaded());

			physics.forward();
			physics.step();

			mjData* snapshot_data = nullptr;
			const mjModel* snapshot_model = nullptr;
			double snapshot_time = 0.0;
			REQUIRE(physics.alloc_render_snapshot(snapshot_data, snapshot_model, snapshot_time));
			REQUIRE(snapshot_model != nullptr);
			REQUIRE(snapshot_data != nullptr);

			const mjModel* copied_model = nullptr;
			double copied_time = 0.0;
			REQUIRE(physics.copy_render_snapshot(snapshot_data, copied_model, copied_time));
			REQUIRE(copied_model == physics.model());

			physics.destroy_render_snapshot(snapshot_data);
			REQUIRE(snapshot_data == nullptr);

			physics.unload();
			REQUIRE_FALSE(physics.is_loaded());
		}
	}
#else
	TEST_CASE("Unit/Systems/MuJoCoPhysics")
	{
		SUCCEED();
	}
#endif
} // namespace robotick::tests
