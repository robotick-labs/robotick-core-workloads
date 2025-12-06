// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#include "robotick/api.h"
#include "robotick/framework/Engine.h"
#include "robotick/framework/concurrency/Atomic.h"
#include "robotick/framework/concurrency/Thread.h"
#include "robotick/framework/utils/TypeId.h"

#include <catch2/catch_all.hpp>
#include <vector>

using namespace robotick;

namespace
{
	// === CountingWorkload ===

	struct CountingWorkload
	{
		AtomicValue<int> tick_count{0};
		float last_dt{0};

		void tick(const TickInfo& tick_info)
		{
			last_dt = tick_info.delta_time;
			tick_count.fetch_add(1);
		}
	};
	ROBOTICK_REGISTER_WORKLOAD(CountingWorkload);

	// === SlowWorkload ===

	struct SlowWorkload
	{
		AtomicValue<int> tick_count{0};
		void tick(const TickInfo&)
		{
			tick_count.fetch_add(1);
			Thread::sleep_ms(30);
		}
	};
	ROBOTICK_REGISTER_WORKLOAD(SlowWorkload);

	struct ThreadAwareWorkload
	{
		Thread::ThreadId start_thread = 0;
		Thread::ThreadId tick_thread = 0;
		AtomicValue<int> tick_count{0};

		void start(float) { start_thread = Thread::get_current_thread_id(); }
		void tick(const TickInfo&)
		{
			if (tick_count.fetch_add(1) == 0)
			{
				tick_thread = Thread::get_current_thread_id();
			}
		}
	};
	ROBOTICK_REGISTER_WORKLOAD(ThreadAwareWorkload);

} // namespace

TEST_CASE("Unit/Workloads/SyncedGroupWorkload")
{
	SECTION("All children tick in parallel")
	{
		const TickInfo tick_info = TICK_INFO_FIRST_10MS_100HZ;
		const float tick_rate_hz = 1.0f / tick_info.delta_time;
		const int tick_count = 5;

		Model model;
		const WorkloadSeed& a = model.add("CountingWorkload", "a").set_tick_rate_hz(tick_rate_hz);
		const WorkloadSeed& b = model.add("CountingWorkload", "b").set_tick_rate_hz(tick_rate_hz);
		const WorkloadSeed& group_seed = model.add("SyncedGroupWorkload", "group").set_children({&a, &b}).set_tick_rate_hz(tick_rate_hz);
		model.set_root_workload(group_seed);

		Engine engine;
		engine.load(model);

		const auto& info = *engine.find_instance_info(group_seed.unique_name);
		auto* group_ptr = info.get_ptr(engine);

		REQUIRE(group_ptr != nullptr);

		info.type->get_workload_desc()->start_fn(group_ptr, tick_rate_hz);

		auto* wa = engine.find_instance<CountingWorkload>(a.unique_name);
		auto* wb = engine.find_instance<CountingWorkload>(b.unique_name);

		for (int i = 0; i < tick_count; ++i)
		{
			info.type->get_workload_desc()->tick_fn(group_ptr, tick_info);

			const auto sleep_ms = static_cast<uint32_t>(tick_info.delta_time * 1000.0f);
			Thread::sleep_ms(sleep_ms > 0 ? sleep_ms : 1);

			// Confirm each child has exactly i + 1 ticks after this iteration
			CHECK(wa->tick_count.load() == i + 1);
			CHECK(wb->tick_count.load() == i + 1);
		}

		info.type->get_workload_desc()->stop_fn(group_ptr);
	}

	SECTION("Child busy flags skip ticks")
	{
		const TickInfo tick_info = TICK_INFO_FIRST_10MS_100HZ;
		const float tick_rate_hz = 1.0f / tick_info.delta_time;
		constexpr int num_ticks = 5;

		Model model;
		const WorkloadSeed& s1 = model.add("SlowWorkload", "s1").set_tick_rate_hz(tick_rate_hz);
		const WorkloadSeed& s2 = model.add("SlowWorkload", "s2").set_tick_rate_hz(tick_rate_hz);
		const WorkloadSeed& group_seed = model.add("SyncedGroupWorkload", "group").set_children({&s1, &s2}).set_tick_rate_hz(tick_rate_hz);
		model.set_root_workload(group_seed);

		Engine engine;
		engine.load(model);

		const auto& group_info = *engine.find_instance_info(group_seed.unique_name);
		auto* group_ptr = group_info.get_ptr(engine);

		REQUIRE(group_ptr != nullptr);

		group_info.type->get_workload_desc()->start_fn(group_ptr, tick_rate_hz);

		for (int i = 0; i < num_ticks; ++i)
		{
			group_info.type->get_workload_desc()->tick_fn(group_ptr, tick_info);
			Thread::sleep_ms(10); // Let threads get through, but not enough for all to finish
		}

		group_info.type->get_workload_desc()->stop_fn(group_ptr);

		const auto* w1 = engine.find_instance<SlowWorkload>(s1.unique_name);
		const auto* w2 = engine.find_instance<SlowWorkload>(s2.unique_name);

		INFO("Tick count s1: " << w1->tick_count.load());
		INFO("Tick count s2: " << w2->tick_count.load());

		// We expect 5 ticks issued, 10ms between ticks, and slow-job taking 30ms → can only respond to every 3rd
		CHECK(w1->tick_count.load() == 2);
		CHECK(w2->tick_count.load() == 2);
	}

	SECTION("tick() passes real time_delta (child thread measures time elapsed since last actionable tick)")
	{
		const TickInfo tick_info = TICK_INFO_FIRST_10MS_100HZ;
		const float tick_rate_hz = 1.0f / tick_info.delta_time;

		Model model;
		const WorkloadSeed& h = model.add("CountingWorkload", "ticky").set_tick_rate_hz(tick_rate_hz);
		const WorkloadSeed& group_seed = model.add("SyncedGroupWorkload", "group").set_children({&h}).set_tick_rate_hz(tick_rate_hz);
		model.set_root_workload(group_seed);

		Engine engine;
		engine.load(model);

		const WorkloadInstanceInfo& group_info = *engine.find_instance_info(group_seed.unique_name);
		const WorkloadInstanceInfo& child_info = *engine.find_instance_info(h.unique_name);

		auto* counting = static_cast<CountingWorkload*>((void*)child_info.get_ptr(engine));

		auto* group_ptr = group_info.get_ptr(engine);

		group_info.type->get_workload_desc()->start_fn(group_ptr, tick_rate_hz);

		Thread::sleep_ms(20);
		group_info.type->get_workload_desc()->tick_fn(group_ptr, tick_info);

		Thread::sleep_ms(40);
		const float first_dt = counting->last_dt;

		group_info.type->get_workload_desc()->tick_fn(group_ptr, tick_info);

		Thread::sleep_ms(10); // give a bit if time for the tick to complete
		group_info.type->get_workload_desc()->stop_fn(group_ptr);

		INFO("First time_delta (expected 0.02sec): " << first_dt);
		CHECK_THAT(first_dt, Catch::Matchers::WithinAbs(0.02, 0.005)); // allow ±5ms - since we're not allowing for code-duration when sleeping above

		INFO("Last time_delta (expected 0.04sec): " << counting->last_dt);
		CHECK_THAT(counting->last_dt, Catch::Matchers::WithinAbs(0.04, 0.005)); // (ditto)
	}

	SECTION("Child allowed to run at slower fixed tick rate than group")
	{
		const TickInfo group_tick_info = TICK_INFO_FIRST_10MS_100HZ;
		const float group_tick_rate_hz = 1.0f / group_tick_info.delta_time;
		const float child_tick_rate_hz = 1.0f / TICK_INFO_FIRST_100MS_10HZ.delta_time; // child wants to tick 10x slower than group - we should let it

		Model model;
		const WorkloadSeed& h = model.add("CountingWorkload", "slower").set_tick_rate_hz(child_tick_rate_hz);
		const WorkloadSeed& group_seed = model.add("SyncedGroupWorkload", "group").set_children({&h}).set_tick_rate_hz(group_tick_rate_hz);
		model.set_root_workload(group_seed);

		Engine engine;
		engine.load(model);

		const auto& group_info = *engine.find_instance_info(group_seed.unique_name);
		const auto& child_info = *engine.find_instance_info(h.unique_name);

		auto* counting = static_cast<CountingWorkload*>((void*)child_info.get_ptr(engine));
		auto* group_ptr = group_info.get_ptr(engine);

		group_info.type->get_workload_desc()->start_fn(group_ptr, group_tick_rate_hz);

		constexpr int num_group_ticks = 10; // total time: 100ms
		for (int i = 0; i < num_group_ticks; ++i)
		{
			group_info.type->get_workload_desc()->tick_fn(group_ptr, group_tick_info);
			Thread::sleep_ms(10);
		}

		Thread::sleep_ms(20); // allow child time to complete final tick
		group_info.type->get_workload_desc()->stop_fn(group_ptr);

		INFO("Child tick count (expected ~2): " << counting->tick_count.load());
		CHECK(counting->tick_count.load() == 2);
	}

	SECTION("Child start executes on same thread as child tick")
	{
		const TickInfo tick_info = TICK_INFO_FIRST_10MS_100HZ;
		const float tick_rate_hz = 1.0f / tick_info.delta_time;

		Model model;
		const WorkloadSeed& child_seed = model.add("ThreadAwareWorkload", "threadedChild").set_tick_rate_hz(tick_rate_hz);
		const WorkloadSeed& group_seed = model.add("SyncedGroupWorkload", "group").set_children({&child_seed}).set_tick_rate_hz(tick_rate_hz);
		model.set_root_workload(group_seed);

		Engine engine;
		engine.load(model);

		const auto& group_info = *engine.find_instance_info(group_seed.unique_name);
		auto* group_ptr = group_info.get_ptr(engine);
		REQUIRE(group_ptr != nullptr);

		group_info.type->get_workload_desc()->start_fn(group_ptr, tick_rate_hz);

		for (int i = 0; i < 3; ++i)
		{
			group_info.type->get_workload_desc()->tick_fn(group_ptr, tick_info);
			Thread::sleep_ms(10);
		}

		group_info.type->get_workload_desc()->stop_fn(group_ptr);

		const auto* child = engine.find_instance<ThreadAwareWorkload>(child_seed.unique_name);
		REQUIRE(child != nullptr);
		REQUIRE(child->tick_count.load() > 0);
		CHECK(child->start_thread == child->tick_thread);
		CHECK(child->start_thread != 0);
	}
}
