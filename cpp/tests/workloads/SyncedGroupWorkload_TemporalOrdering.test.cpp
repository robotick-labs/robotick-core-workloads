// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#include "robotick/framework/Engine.h"
#include "robotick/framework/concurrency/Atomic.h"
#include "robotick/framework/concurrency/Thread.h"
#include "robotick/framework/containers/FixedVector.h"
#include "robotick/framework/time/Clock.h"

#include <catch2/catch_all.hpp>

namespace robotick::test
{
	namespace
	{
		struct SenderOut
		{
			int output = 0;
		};
		ROBOTICK_REGISTER_STRUCT_BEGIN(SenderOut)
		ROBOTICK_STRUCT_FIELD(SenderOut, int, output)
		ROBOTICK_REGISTER_STRUCT_END(SenderOut)

		struct ReceiverIn
		{
			int input = 0;
		};
		ROBOTICK_REGISTER_STRUCT_BEGIN(ReceiverIn)
		ROBOTICK_STRUCT_FIELD(ReceiverIn, int, input)
		ROBOTICK_REGISTER_STRUCT_END(ReceiverIn)

		struct SenderWorkload
		{
			SenderOut outputs;
			void tick(const TickInfo&) { outputs.output++; }
		};
		ROBOTICK_REGISTER_WORKLOAD(SenderWorkload, void, void, SenderOut)

		struct ReceiverWorkload
		{
			ReceiverIn inputs;
			FixedVector<int, 2048> received;
			void tick(const TickInfo&)
			{
				if (!received.full())
					received.add(inputs.input);
			}
		};
		ROBOTICK_REGISTER_WORKLOAD(ReceiverWorkload, void, ReceiverIn)
	} // namespace

	TEST_CASE("Unit/Framework/Data/Connection/SyncedGroupWorklaod")
	{
		SECTION("Data connections are propagated correctly")
		{
			Model model;
			const float tick_rate = 100.0f;

			const WorkloadSeed& sender = model.add("SenderWorkload", "sender").set_tick_rate_hz(tick_rate);
			const WorkloadSeed& receiver = model.add("ReceiverWorkload", "receiver").set_tick_rate_hz(tick_rate);
			const WorkloadSeed& group_seed = model.add("SyncedGroupWorkload", "group").set_children({&sender, &receiver}).set_tick_rate_hz(tick_rate);

			model.connect("sender.outputs.output", "receiver.inputs.input");
			model.set_telemetry_port(7999);
			model.set_root_workload(group_seed);

			Engine engine;
			engine.load(model);

			AtomicFlag stop_after_next_tick_flag{false};

			struct RunnerContext
			{
				Engine* engine = nullptr;
				AtomicFlag* stop_flag = nullptr;
				static void entry(void* arg)
				{
					auto* ctx = static_cast<RunnerContext*>(arg);
					ctx->engine->run(*ctx->stop_flag);
				}
			};

			RunnerContext runner_ctx{&engine, &stop_after_next_tick_flag};
			Thread runner(RunnerContext::entry, &runner_ctx, "synced-group-test");

			Thread::sleep_ms(1000);
			stop_after_next_tick_flag.set(true);
			// runner ~Thread() joins automatically

			const auto& receiver_info = *engine.find_instance_info(receiver.unique_name);
			auto* receiver_workload = static_cast<ReceiverWorkload*>((void*)receiver_info.get_ptr(engine));

			const auto received_size = receiver_workload->received.size();
			REQUIRE(received_size > 10);

			size_t num_errors = 0;

			for (size_t i = 1; i < received_size; ++i)
			{
				const int current = receiver_workload->received[i];
				const int prev = receiver_workload->received[i - 1];
				INFO("Received[" << i << "] = " << current);
				num_errors += (current == prev + 1) ? 0 : 1;
			}

			REQUIRE(num_errors < 5); // instrumentation adds some jitter, so tolerating a couple extra gaps
		}
	}

} // namespace robotick::test
