// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#include "robotick/systems/MuJoCoPhysics.h"
#include "robotick/framework/concurrency/Atomic.h"
#include "robotick/framework/concurrency/Thread.h"

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

		SECTION("Snapshot copy waits for physics lock before touching MuJoCo state")
		{
			MuJoCoPhysics physics;
			REQUIRE(physics.load_from_xml(kMinimalModelPath));
			REQUIRE(physics.is_loaded());

			mjData* snapshot_data = nullptr;
			const mjModel* snapshot_model = nullptr;
			double snapshot_time = 0.0;
			REQUIRE(physics.alloc_render_snapshot(snapshot_data, snapshot_model, snapshot_time));
			REQUIRE(snapshot_data != nullptr);

			// Spawn a copy operation while we hold the MuJoCo lock; it should not complete until we unlock.
			AtomicValue<bool> copy_started{false};
			AtomicValue<bool> copy_done{false};
			AtomicValue<bool> copy_ok{false};

			struct CopyThreadArgs
			{
				MuJoCoPhysics* physics = nullptr;
				mjData* snapshot_data = nullptr;
				AtomicValue<bool>* copy_started = nullptr;
				AtomicValue<bool>* copy_done = nullptr;
				AtomicValue<bool>* copy_ok = nullptr;
			};

			CopyThreadArgs args;
			args.physics = &physics;
			args.snapshot_data = snapshot_data;
			args.copy_started = &copy_started;
			args.copy_done = &copy_done;
			args.copy_ok = &copy_ok;

			// Hold the lock to block the copy thread inside MuJoCoPhysics::copy_render_snapshot.
			auto lock = physics.lock();

			Thread copy_thread(
				[](void* raw)
				{
					auto* ctx = static_cast<CopyThreadArgs*>(raw);
					ctx->copy_started->store(true);
					const mjModel* copied_model = nullptr;
					double copied_time = 0.0;
					const bool ok = ctx->physics->copy_render_snapshot(ctx->snapshot_data, copied_model, copied_time);
					ctx->copy_ok->store(ok);
					ctx->copy_done->store(true);
				},
				&args,
				"mujoco_copy",
				-1);

			// Wait until the copy thread begins, then verify it cannot finish while the lock is held.
			for (int i = 0; i < 50 && !copy_started.load(); ++i)
				Thread::sleep_ms(1);

			Thread::sleep_ms(5);
			CHECK_FALSE(copy_done.load());

			// Release the lock and ensure the copy completes successfully.
			lock.unlock();
			copy_thread.join();

			CHECK(copy_done.load());
			CHECK(copy_ok.load());

			physics.destroy_render_snapshot(snapshot_data);
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
