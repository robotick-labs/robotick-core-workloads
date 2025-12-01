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
		bool enable_tls = false;
		uint8_t publish_qos = 1;
		uint8_t subscribe_qos = 1;
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

	struct MqttClientOutputs
	{
		MqttClient::HealthMetrics health{};
		MqttClient::BackpressureStats backpressure{};
		MqttFieldSync::Metrics field_sync_metrics{};
		MqttOpResult last_subscribe = MqttOpResult::Success;
		bool connected = false;
	};

	struct MqttClientWorkload
	{
		MqttClientConfig config;
		MqttClientOutputs outputs;

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
			mqtt_client->set_tls_enabled(config.enable_tls);
			mqtt_client->set_qos(config.publish_qos, config.subscribe_qos);
			if (!mqtt_client->connect())
			{
				ROBOTICK_WARNING("MqttClientWorkload - initial MQTT connect failed (proceeding, will retry on tick).");
			}

			// 2. Create MqttFieldSync
			FixedString64 root_ns(config.root_topic_namespace.c_str());
			auto field_sync = std_approved::make_unique<MqttFieldSync>(*const_cast<Engine*>(state->engine), root_ns.c_str(), *mqtt_client);

			state->mqtt = robotick::move(mqtt_client);
			state->field_sync = robotick::move(field_sync);
			outputs.connected = state->mqtt->is_connected();
			outputs.health = state->mqtt->get_health_metrics();
			outputs.backpressure = state->mqtt->get_backpressure_stats();
		}

		void start(float)
		{
			// Subscribe and initial sync
			if (state->field_sync)
			{
				const MqttOpResult sub_result = state->field_sync->subscribe_and_sync_startup();
				outputs.last_subscribe = sub_result;
				outputs.field_sync_metrics = state->field_sync->get_metrics();
			}
		}

		void tick(const TickInfo&)
		{
			if (!state->field_sync || !state->mqtt)
				return;

			state->field_sync->apply_control_updates();
			state->field_sync->publish_state_fields();
			outputs.field_sync_metrics = state->field_sync->get_metrics();

			state->mqtt->poll();
			outputs.connected = state->mqtt->is_connected();
			outputs.health = state->mqtt->get_health_metrics();
			outputs.backpressure = state->mqtt->get_backpressure_stats();
		}
	};

} // namespace robotick

#endif
