// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#include "robotick/systems/MqttFieldSync.h"
#include "robotick/framework/Engine.h"
#include "robotick/framework/containers/Map.h"
#include "robotick/framework/data/Blackboard.h"
#include "robotick/framework/data/WorkloadsBuffer.h"
#include "robotick/framework/utils/TypeId.h"
#include "robotick/framework/utils/WorkloadFieldsIterator.h"

#include <catch2/catch_all.hpp>
#include <nlohmann/json.hpp>
#include <cstring>

namespace robotick::test
{

	namespace
	{
		struct TestInputs
		{
			int value = 7;
			FixedString64 text = "abc";
			Blackboard blackboard;
		};
		ROBOTICK_REGISTER_STRUCT_BEGIN(TestInputs)
		ROBOTICK_STRUCT_FIELD(TestInputs, int, value)
		ROBOTICK_STRUCT_FIELD(TestInputs, FixedString64, text)
		ROBOTICK_STRUCT_FIELD(TestInputs, Blackboard, blackboard)
		ROBOTICK_REGISTER_STRUCT_END(TestInputs)

		struct TestState
		{
			HeapVector<FieldDescriptor> fields;
		};

		struct TestWorkload
		{
			TestInputs inputs;
			State<TestState> state;

			void pre_load()
			{
				state->fields.initialize(2);
				state->fields[0] = FieldDescriptor{"flag", GET_TYPE_ID(int)};
				state->fields[1] = FieldDescriptor{"ratio", GET_TYPE_ID(double)};
				inputs.blackboard.initialize_fields(state->fields);
			}

			void load()
			{
				inputs.blackboard.set("flag", 1);
				inputs.blackboard.set("ratio", 0.5);
			}
		};
		ROBOTICK_REGISTER_WORKLOAD(TestWorkload, void, TestInputs, void)

		struct DummyMqttClient : public IMqttClient
		{
			Map<FixedString256, FixedString256, 128> retained;
			MqttOpResult subscribe_result = MqttOpResult::Success;
			Function<MqttOpResult(const char*, const char*)> publish_override;

			bool connect() override { return true; }
			MqttOpResult subscribe(const char* /*topic*/, int /*qos*/ = 1) override { return subscribe_result; }

			MqttOpResult publish(const char* topic, const char* payload, bool retain = true) override
			{
				if (!retain)
					return MqttOpResult::Success;

				if (publish_override)
				{
					const MqttOpResult overridden = publish_override(topic, payload);
					if (overridden != MqttOpResult::Success)
						return overridden;
				}

				const FixedString256 key = topic ? FixedString256(topic) : FixedString256("");
				const FixedString256 value = payload ? FixedString256(payload) : FixedString256("");
				retained.insert(key, value);
				return MqttOpResult::Success;
			}

			bool has_retained(const char* topic) const
			{
				const FixedString256 key = topic ? FixedString256(topic) : FixedString256("");
				return retained.find(key) != nullptr;
			}

			void clear_retained() { retained.clear(); }

			void set_callback(Function<void(const char*, const char*)>) override {}

			void set_publish_override(Function<MqttOpResult(const char*, const char*)> fn) { publish_override = robotick::move(fn); }
			void set_subscribe_result(MqttOpResult result) { subscribe_result = result; }
		};
	} // namespace

	TEST_CASE("Unit/Framework/Data/MqttFieldSync")
	{
		SECTION("MqttFieldSync can publish state and control fields")
		{
			Model model;
			const WorkloadSeed& test_workload_seed = model.add("TestWorkload", "W1").set_tick_rate_hz(1.0f);
			model.set_root_workload(test_workload_seed);

			Engine engine;
			engine.load(model);

			// initialize our input fields & blackboard-fields:
			const auto& info = *engine.find_instance_info(test_workload_seed.unique_name);
			auto* test_workload_ptr = static_cast<TestWorkload*>((void*)info.get_ptr(engine));
			test_workload_ptr->inputs.value = 42;
			test_workload_ptr->inputs.blackboard.set("flag", 2);
			test_workload_ptr->inputs.blackboard.set("ratio", 3.14);

			WorkloadsBuffer mirror_buf;
			mirror_buf.create_mirror_from(engine.get_workloads_buffer());

			DummyMqttClient dummy_client;
			FixedString64 root_topic_name = "robotick";
			MqttFieldSync sync(engine, root_topic_name.c_str(), dummy_client);

			sync.subscribe_and_sync_startup();

			// Check retained messages contain both state and control for inputs
			CHECK(dummy_client.has_retained("robotick/state/W1/inputs/value"));
			CHECK(dummy_client.has_retained("robotick/state/W1/inputs/text"));
			CHECK(dummy_client.has_retained("robotick/state/W1/inputs/blackboard/flag"));
			CHECK(dummy_client.has_retained("robotick/state/W1/inputs/blackboard/ratio"));

			CHECK(dummy_client.has_retained("robotick/control/W1/inputs/value"));
			CHECK(dummy_client.has_retained("robotick/control/W1/inputs/text"));
			CHECK(dummy_client.has_retained("robotick/control/W1/inputs/blackboard/flag"));
			CHECK(dummy_client.has_retained("robotick/control/W1/inputs/blackboard/ratio"));

			// Clear retained and test publish_state_fields only
			dummy_client.clear_retained();
			sync.publish_state_fields();
			CHECK(dummy_client.has_retained("robotick/state/W1/inputs/value"));
			CHECK_FALSE(dummy_client.has_retained("robotick/control/W1/inputs/value"));
		}

		SECTION("MqttFieldSync metrics capture subscribe failures")
		{
			Model model;
			const WorkloadSeed& test_workload_seed = model.add("TestWorkload", "W1");
			model.set_root_workload(test_workload_seed);

			Engine engine;
			engine.load(model);

			DummyMqttClient dummy_client;
			dummy_client.set_subscribe_result(MqttOpResult::Dropped);

			MqttFieldSync sync(engine, "robotick", dummy_client);
			const MqttOpResult start_result = sync.subscribe_and_sync_startup();
			CHECK(start_result == MqttOpResult::Dropped);

			const auto& metrics = sync.get_metrics();
			CHECK(metrics.subscribe_failures == 1);
			CHECK(metrics.last_subscribe_result == MqttOpResult::Dropped);
		}

		SECTION("MqttFieldSync metrics capture publish failures")
		{
			Model model;
			const WorkloadSeed& test_workload_seed = model.add("TestWorkload", "W1");
			model.set_root_workload(test_workload_seed);

			Engine engine;
			engine.load(model);

			DummyMqttClient dummy_client;
			bool state_failed = false;
			bool control_failed = false;
			dummy_client.set_publish_override(
				[&](const char* topic, const char*) -> MqttOpResult
				{
					if (!state_failed && ::strstr(topic, "/state/"))
					{
						state_failed = true;
						return MqttOpResult::Error;
					}
					if (!control_failed && ::strstr(topic, "/control/"))
					{
						control_failed = true;
						return MqttOpResult::Dropped;
					}
					return MqttOpResult::Success;
				});

			MqttFieldSync sync(engine, "robotick", dummy_client);
			sync.subscribe_and_sync_startup();
			sync.publish_fields(engine, engine.get_workloads_buffer(), true);

			const auto& metrics = sync.get_metrics();
			CHECK(metrics.state_publish_failures == 1);
			CHECK(metrics.control_publish_failures == 1);
			CHECK(metrics.last_state_result == MqttOpResult::Success);
			CHECK(metrics.last_control_result == MqttOpResult::Success);
		}

		SECTION("MqttFieldSync can apply control updates")
		{
			Model model;
			const WorkloadSeed& test_workload_seed = model.add("TestWorkload", "W2").set_tick_rate_hz(1.0f);
			model.set_root_workload(test_workload_seed);

			Engine engine;
			engine.load(model);

			const auto& info = *engine.find_instance_info(test_workload_seed.unique_name);
			auto* test_workload_ptr = static_cast<TestWorkload*>((void*)info.get_ptr(engine));

			DummyMqttClient dummy_client;
			FixedString64 root_topic_name = "robotick";
			MqttFieldSync sync(engine, root_topic_name.c_str(), dummy_client);

			nlohmann::json json_val = 99;
			nlohmann::json json_subint = 5;
			sync.queue_control_topic("robotick/control/W2/inputs/value", json_val);
			sync.queue_control_topic("robotick/control/W2/inputs/blackboard/flag", json_subint);

			sync.apply_control_updates();

			int val = test_workload_ptr->inputs.value;
			int flag = test_workload_ptr->inputs.blackboard.get<int>("flag");

			CHECK(val == 99);
			CHECK(flag == 5);
		}
	}

} // namespace robotick::test
