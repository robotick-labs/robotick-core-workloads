// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#include "robotick/systems/MuJoCoRenderContext.h"
#include "robotick/systems/MuJoCoPhysics.h"

#include <catch2/catch_test_macros.hpp>

namespace robotick::tests
{
#if defined(ROBOTICK_PLATFORM_DESKTOP) || defined(ROBOTICK_PLATFORM_LINUX)
	namespace
	{
		constexpr char kModelPath[] = ROBOTICK_CORE_ROOT "/cpp/tests/data/mujoco/minimal.xml";
	} // namespace

	TEST_CASE("Unit/Systems/MuJoCoRenderContext/RenderPNG")
	{
		MuJoCoPhysics physics;
		REQUIRE(physics.load_from_xml(kModelPath));

		mjData* snapshot_data = nullptr;
		const mjModel* snapshot_model = nullptr;
		double snapshot_time = 0.0;
		REQUIRE(physics.alloc_render_snapshot(snapshot_data, snapshot_model, snapshot_time));
		REQUIRE(snapshot_model != nullptr);
		REQUIRE(snapshot_data != nullptr);

		MuJoCoRenderContext context;
		if (!context.init(snapshot_model, 64, 48))
		{
			MuJoCoPhysics::destroy_snapshot(snapshot_data);
			SKIP("MuJoCo render context init failed (likely headless GL)");
		}

		ImagePng128k png;
		if (!context.render_to_png(snapshot_model, snapshot_data, "", png))
		{
			MuJoCoPhysics::destroy_snapshot(snapshot_data);
			SKIP("MuJoCo render failed (likely headless GL)");
		}

		REQUIRE(png.size() > 0);

		MuJoCoPhysics::destroy_snapshot(snapshot_data);
	}
#else
	TEST_CASE("Unit/Systems/MuJoCoRenderContext/SkipNonDesktop")
	{
		SUCCEED();
	}
#endif
} // namespace robotick::tests
