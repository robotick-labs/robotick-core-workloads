// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#if defined(ROBOTICK_PLATFORM_DESKTOP) || defined(ROBOTICK_PLATFORM_LINUX)

#include "robotick/api.h"
#include "robotick/framework/Engine.h"
#include "robotick/framework/data/WorkloadsBuffer.h"
#include "robotick/framework/memory/Memory.h"
#include "robotick/framework/memory/StdApproved.h"
#include "robotick/framework/strings/FixedString.h"
#include "robotick/systems/MqttClient.h"
#include "robotick/systems/MqttFieldSync.h"

namespace robotick
{
	//----------------------------------------------------------------------
	// Config, Inputs, Outputs
	//----------------------------------------------------------------------

	struct MqttClientConfig
	{
		FixedString64 broker_url = "mqtt://localhost";
		uint16_t broker_mqtt_port = 1883;
		FixedString64 root_topic_namespace = "robotick";
	};

	//----------------------------------------------------------------------
	// Internal State
	//----------------------------------------------------------------------

	class MqttClientWorkloadState
	{
	  public:
		std_approved::unique_ptr<MqttClient> mqtt;
		std_approved::unique_ptr<MqttFieldSync> field_sync;
		const Engine* engine = nullptr;
	};

	//----------------------------------------------------------------------
	// Workload
	//----------------------------------------------------------------------

	struct MqttClientWorkload
	{
		MqttClientConfig config;

		State<MqttClientWorkloadState> state;

		void set_engine(const Engine& engine_in) { state->engine = &engine_in; }

		void load()
		{
			ROBOTICK_ASSERT_MSG(state->engine != nullptr, "Engine must be set before load()");

			// 1. Create and connect MQTT client

			FixedString64 broker_url(config.broker_url.c_str());
			const size_t broker_url_len = broker_url.length();
			if (broker_url_len > 0)
			{
				char* url_data = broker_url.str();
				if (url_data[broker_url_len - 1] == '/')
				{
					url_data[broker_url_len - 1] = '\0';
				}
			}

			FixedString128 broker;
			broker.format("%s:%u", broker_url.c_str(), config.broker_mqtt_port);

			FixedString64 client_id("robotick::MqttClientWorkload");
			auto mqtt_client = std_approved::make_unique<MqttClient>(broker.c_str(), client_id.c_str());
			mqtt_client->connect();

			// 2. Create MqttFieldSync
			FixedString64 root_ns(config.root_topic_namespace.c_str());
			auto field_sync = std_approved::make_unique<MqttFieldSync>(*const_cast<Engine*>(state->engine), root_ns.c_str(), *mqtt_client);

			state->mqtt = robotick::move(mqtt_client);
			state->field_sync = robotick::move(field_sync);
		}

		void start(float)
		{
			// Subscribe and initial sync
			state->field_sync->subscribe_and_sync_startup();
		}

		void tick(const TickInfo&)
		{
			state->field_sync->apply_control_updates();
			state->field_sync->publish_state_fields();
		}
	};

} // namespace robotick

#endif
