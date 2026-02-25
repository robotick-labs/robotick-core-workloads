// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#include "robotick/systems/MuJoCoSceneRegistry.h"

#include <catch2/catch_test_macros.hpp>

namespace robotick::tests
{
#if defined(ROBOTICK_PLATFORM_DESKTOP) || defined(ROBOTICK_PLATFORM_LINUX)
	namespace
	{
		// Minimal MuJoCo scene with empty worldbody; used for registry and snapshot smoke tests.
		constexpr char kMinimalModelPath[] = ROBOTICK_CORE_ROOT "/cpp/tests/data/mujoco/minimal.xml";
	} // namespace

	TEST_CASE("Unit/Systems/MuJoCoSceneRegistry")
	{
		SECTION("Registers a scene and handles snapshot lifecycle")
		{
			MuJoCoPhysics physics;
			REQUIRE(physics.load_from_xml(kMinimalModelPath));

			MuJoCoSceneRegistry& registry = MuJoCoSceneRegistry::get();
			const uint32_t scene_id = registry.register_scene(&physics);
			REQUIRE(scene_id != 0);
			REQUIRE(registry.is_valid(scene_id));

			mjData* snapshot_data = nullptr;
			const mjModel* snapshot_model = nullptr;
			double snapshot_time = 0.0;
			REQUIRE(registry.alloc_render_snapshot(scene_id, snapshot_data, snapshot_model, snapshot_time));
			REQUIRE(snapshot_model != nullptr);
			REQUIRE(snapshot_data != nullptr);

			const mjModel* copied_model = nullptr;
			double copied_time = 0.0;
			REQUIRE(registry.copy_render_snapshot(scene_id, snapshot_data, copied_model, copied_time));
			REQUIRE(copied_model == snapshot_model);

			registry.destroy_render_snapshot(snapshot_data);

			registry.unregister_scene(scene_id);
			REQUIRE_FALSE(registry.is_valid(scene_id));
		}

		SECTION("Rejects operations on invalid handles")
		{
			MuJoCoSceneRegistry& registry = MuJoCoSceneRegistry::get();
			const uint32_t invalid_id = 0xdeadbeefu;
			REQUIRE_FALSE(registry.is_valid(invalid_id));
			mjData* snapshot_data = nullptr;
			const mjModel* snapshot_model = nullptr;
			double snapshot_time = 0.0;
			REQUIRE_FALSE(registry.alloc_render_snapshot(invalid_id, snapshot_data, snapshot_model, snapshot_time));
			REQUIRE(snapshot_data == nullptr);
			REQUIRE(snapshot_model == nullptr);
		}
	}
#else
	TEST_CASE("Unit/Systems/MuJoCoSceneRegistry")
	{
		SUCCEED();
	}
#endif
} // namespace robotick::tests
