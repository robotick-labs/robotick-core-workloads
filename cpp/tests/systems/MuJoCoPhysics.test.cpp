// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#include "robotick/systems/MuJoCoPhysics.h"

#include <catch2/catch_test_macros.hpp>

namespace robotick::tests
{
#if defined(ROBOTICK_PLATFORM_DESKTOP) || defined(ROBOTICK_PLATFORM_LINUX)
	namespace
	{
		constexpr char kModelPath[] = ROBOTICK_CORE_ROOT "/cpp/tests/data/mujoco/minimal.xml";
	} // namespace

	TEST_CASE("Unit/Systems/MuJoCoPhysics/LoadInvalidPath")
	{
		MuJoCoPhysics physics;
		REQUIRE_FALSE(physics.load_from_xml("does_not_exist.xml"));
	}

	TEST_CASE("Unit/Systems/MuJoCoPhysics/LoadAndSnapshot")
	{
		MuJoCoPhysics physics;
		REQUIRE(physics.load_from_xml(kModelPath));
		REQUIRE(physics.is_loaded());

		physics.forward();
		physics.step();

		MuJoCoRenderSnapshot snapshot = physics.get_render_snapshot();
		REQUIRE(snapshot.model != nullptr);
		REQUIRE(snapshot.data != nullptr);

		physics.free_render_snapshot(snapshot);
		REQUIRE(snapshot.data == nullptr);

		physics.unload();
		REQUIRE_FALSE(physics.is_loaded());
	}
#else
	TEST_CASE("Unit/Systems/MuJoCoPhysics/SkipNonDesktop")
	{
		SUCCEED();
	}
#endif
} // namespace robotick::tests
