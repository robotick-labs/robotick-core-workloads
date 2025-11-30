// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#include "robotick/api.h"
#include "robotick/framework/Engine.h"
#include "robotick/framework/concurrency/Thread.h"
#include "robotick/framework/utils/TypeId.h"

#include <atomic>
#include <catch2/catch_all.hpp>

using namespace robotick;

namespace
{
	// === DummyTickingWorkload ===

	struct DummyTickingWorkload
	{
		inline static int tick_count = 0;
		void tick(const TickInfo&) { ++tick_count; }
		static void reset() { tick_count = 0; }
	};

	ROBOTICK_REGISTER_WORKLOAD(DummyTickingWorkload);

	// === SlowTickWorkload ===

	struct SlowTickWorkload
	{
		void tick(const TickInfo&) { Thread::sleep_ms(20); }
	};

	ROBOTICK_REGISTER_WORKLOAD(SlowTickWorkload);

	struct ThreadAwareSequencedChild
	{
		Thread::ThreadId start_thread = 0;
		Thread::ThreadId tick_thread = 0;
		int tick_count = 0;

		void start(float) { start_thread = Thread::get_current_thread_id(); }
		void tick(const TickInfo&)
		{
			if (tick_count++ == 0)
			{
				tick_thread = Thread::get_current_thread_id();
			}
		}
	};

	ROBOTICK_REGISTER_WORKLOAD(ThreadAwareSequencedChild);

} // namespace

TEST_CASE("Unit/Workloads/SequencedGroupWorkload")
{
	SECTION("Child ticks are invoked in sequence")
	{
		DummyTickingWorkload::reset();

		Model model;
		const WorkloadSeed& child1 = model.add("DummyTickingWorkload", "child1").set_tick_rate_hz(50.0);
		const WorkloadSeed& child2 = model.add("DummyTickingWorkload", "child2").set_tick_rate_hz(50.0);
		const WorkloadSeed& group = model.add("SequencedGroupWorkload", "group").set_children({&child1, &child2}).set_tick_rate_hz(50.0f);
		model.set_root_workload(group);

		Engine engine;
		engine.load(model);

		const auto& group_info = *engine.find_instance_info(group.unique_name);
		auto* group_ptr = group_info.get_ptr(engine);
		REQUIRE(group_ptr != nullptr);

		const WorkloadDescriptor* workload_desc = group_info.type->get_workload_desc();

		REQUIRE_NOTHROW(workload_desc->start_fn(group_ptr, 50.0f));
		REQUIRE_NOTHROW(workload_desc->tick_fn(group_ptr, TICK_INFO_FIRST_10MS_100HZ));
		REQUIRE_NOTHROW(workload_desc->stop_fn(group_ptr));

		CHECK(DummyTickingWorkload::tick_count == 2);
	}

	SECTION("Overrun logs if exceeded")
	{
		Model model;
		const WorkloadSeed& workload_seed = model.add("SlowTickWorkload", "slow").set_tick_rate_hz(50.0f);
		const WorkloadSeed& group_seed = model.add("SequencedGroupWorkload", "group").set_children({&workload_seed}).set_tick_rate_hz(1000.0f);
		model.set_root_workload(group_seed);

		Engine engine;
		engine.load(model);

		const auto* group_info = engine.find_instance_info(group_seed.unique_name);
		REQUIRE(group_info != nullptr);
		REQUIRE_NOTHROW(
			group_info->type->get_workload_desc()->tick_fn(group_info->get_ptr(engine), TICK_INFO_FIRST_1MS_1KHZ)); // 1ms budget, expect warning log
	}

	SECTION("Child start executes on same thread as child tick")
	{
		Model model;
		const WorkloadSeed& child_seed = model.add("ThreadAwareSequencedChild", "child").set_tick_rate_hz(50.0f);
		const WorkloadSeed& group_seed = model.add("SequencedGroupWorkload", "group").set_children({&child_seed}).set_tick_rate_hz(50.0f);
		model.set_root_workload(group_seed);

		Engine engine;
		engine.load(model);

		const auto& group_info = *engine.find_instance_info(group_seed.unique_name);
		auto* group_ptr = group_info.get_ptr(engine);
		REQUIRE(group_ptr != nullptr);

		const WorkloadDescriptor* desc = group_info.type->get_workload_desc();
		desc->start_fn(group_ptr, 50.0f);
		desc->tick_fn(group_ptr, TICK_INFO_FIRST_10MS_100HZ);
		desc->stop_fn(group_ptr);

		const auto* child = engine.find_instance<ThreadAwareSequencedChild>(child_seed.unique_name);
		REQUIRE(child != nullptr);
		REQUIRE(child->tick_count > 0);
		CHECK(child->start_thread == child->tick_thread);
		CHECK(child->start_thread != 0);
	}
}
