// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "robotick/framework/Engine.h"
#include "robotick/framework/containers/Map.h"
#include "robotick/framework/data/WorkloadsBuffer.h"
#include "robotick/framework/strings/FixedString.h"
#include "robotick/framework/utility/Function.h"
#include "robotick/framework/utils/WorkloadFieldsIterator.h"
#include "robotick/systems/MqttClient.h"

#if defined(ROBOTICK_PLATFORM_DESKTOP)
#include <nlohmann/json.hpp>
#else
namespace nlohmann
{
	class json;
}
#endif

namespace robotick
{
	class MqttFieldSync
	{
	  public:
		// For unit tests: only a publisher lambda
		using PublisherFn = Function<void(const char*, const char*, bool)>;
		using TopicMap = Map<FixedString256, nlohmann::json, 128>;

		/** Constructor for tests (no Engine/IMqttClient) */
		MqttFieldSync(const char* root_ns, PublisherFn publisher);

		/** Constructor for real use: links to Engine and an existing IMqttClient */
		MqttFieldSync(Engine& engine, const char* root_ns, IMqttClient& mqtt_client);

		/** Subscribe to "<root>/control/#" and publish initial fields (state+control) */
		MqttOpResult subscribe_and_sync_startup();

		/** Apply any queued control updates into the Engine’s main buffer */
		void apply_control_updates();

		/** Publish only state fields (no control) to "<root>/state/…" */
		void publish_state_fields();

		/**
		 * Publish all fields under "<root>/state/…" and optionally "<root>/control/…"
		 *   - engine:    the Engine whose fields we iterate
		 *   - buffer:    the snapshot buffer (WorkloadsBuffer) to read from
		 *   - publish_control: if true, also emit to "<root>/control/…"
		 */
		void publish_fields(const Engine& engine, const WorkloadsBuffer& buffer, bool publish_control);

		void queue_control_topic(const char* topic, const nlohmann::json& value);
		struct Metrics
		{
			uint32_t state_publish_failures = 0;
			uint32_t control_publish_failures = 0;
			uint32_t subscribe_failures = 0;
			MqttOpResult last_subscribe_result = MqttOpResult::Success;
			MqttOpResult last_state_result = MqttOpResult::Success;
			MqttOpResult last_control_result = MqttOpResult::Success;
		};
		const Metrics& get_metrics() const { return metrics; }
		void reset_metrics() { metrics = {}; }

	  private:
		FixedString256 root;
		PublisherFn publisher;
		IMqttClient* mqtt_ptr;
		Engine* engine_ptr = nullptr;
		TopicMap last_published;
		TopicMap updated_topics;
		Metrics metrics;

		/** Serialize a single field (by pointer and TypeId) into JSON */
		nlohmann::json serialize(void* ptr, TypeId type);
		void store_topic(TopicMap& table, const char* topic, const nlohmann::json& value);
		bool topic_starts_with(const char* topic, const char* prefix) const;
	};
} // namespace robotick
