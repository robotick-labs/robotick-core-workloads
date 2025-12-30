// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#include "robotick/systems/MuJoCoSceneRegistry.h"

#include <catch2/catch_test_macros.hpp>

namespace robotick::tests
{
#if defined(ROBOTICK_PLATFORM_DESKTOP) || defined(ROBOTICK_PLATFORM_LINUX)
	namespace
	{
		constexpr char kModelPath[] = ROBOTICK_CORE_ROOT "/cpp/tests/data/mujoco/minimal.xml";
	} // namespace

	TEST_CASE("Unit/Systems/MuJoCoSceneRegistry/RegisterAndSnapshot")
	{
		MuJoCoPhysics physics;
		REQUIRE(physics.load_from_xml(kModelPath));

		MuJoCoSceneRegistry& registry = MuJoCoSceneRegistry::get();
		const uint64_t scene_id = registry.register_scene(&physics);
		REQUIRE(scene_id != 0);
		REQUIRE(registry.is_valid(scene_id));

		MuJoCoRenderSnapshot snapshot = registry.get_render_snapshot(scene_id);
		REQUIRE(snapshot.model != nullptr);
		REQUIRE(snapshot.data != nullptr);

		physics.free_render_snapshot(snapshot);
		registry.unregister_scene(scene_id);
		REQUIRE_FALSE(registry.is_valid(scene_id));
	}

	TEST_CASE("Unit/Systems/MuJoCoSceneRegistry/InvalidHandle")
	{
		MuJoCoSceneRegistry& registry = MuJoCoSceneRegistry::get();
		const uint64_t invalid_id = 0xdeadbeefULL;
		REQUIRE_FALSE(registry.is_valid(invalid_id));
		MuJoCoRenderSnapshot snapshot = registry.get_render_snapshot(invalid_id);
		REQUIRE(snapshot.data == nullptr);
		REQUIRE(snapshot.model == nullptr);
	}
#else
	TEST_CASE("Unit/Systems/MuJoCoSceneRegistry/SkipNonDesktop")
	{
		SUCCEED();
	}
#endif
} // namespace robotick::tests
