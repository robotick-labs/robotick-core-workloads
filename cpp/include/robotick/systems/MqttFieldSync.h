// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#pragma once

#if defined(ROBOTICK_PLATFORM_DESKTOP)

#include "robotick/framework/Engine.h"
#include "robotick/framework/containers/Map.h"
#include "robotick/framework/data/WorkloadsBuffer.h"
#include "robotick/framework/strings/FixedString.h"
#include "robotick/framework/utility/Function.h"
#include "robotick/framework/utils/WorkloadFieldsIterator.h"
#include "robotick/systems/MqttClient.h"

#include <nlohmann/json.hpp>

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
		void subscribe_and_sync_startup();

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

	  private:
		FixedString256 root;
		PublisherFn publisher;
		IMqttClient* mqtt_ptr;
		Engine* engine_ptr = nullptr;
		TopicMap last_published;
		TopicMap updated_topics;

		/** Serialize a single field (by pointer and TypeId) into JSON */
		nlohmann::json serialize(void* ptr, TypeId type);
		void store_topic(TopicMap& table, const char* topic, const nlohmann::json& value);
		bool topic_starts_with(const char* topic, const char* prefix) const;
	};
} // namespace robotick

#else // !defined(ROBOTICK_PLATFORM_DESKTOP)

#include <string>
#include <unordered_map>

namespace nlohmann
{
	struct json
	{
	}; // stub implementation for now
} // namespace nlohmann

namespace robotick
{

	class Engine;
	class WorkloadsBuffer;
	class IMqttClient;

	class MqttFieldSync
	{
	  public:
		using PublisherFn = Function<void(const char*, const char*, bool)>;

		inline MqttFieldSync(const char* /*root_ns*/, PublisherFn /*publisher*/) {}
		inline MqttFieldSync(Engine& /*engine*/, const char* /*root_ns*/, IMqttClient& /*mqtt_client*/) {}

		inline void subscribe_and_sync_startup() {}
		inline void apply_control_updates() {}
		inline void publish_state_fields() {}
		inline void publish_fields(const Engine& /*engine*/, const WorkloadsBuffer& /*buffer*/, bool /*publish_control*/) {}

		inline void queue_control_topic(const char* /*topic*/, const nlohmann::json& /*value*/) {}
	};

} // namespace robotick

#endif // #if defined(ROBOTICK_PLATFORM_DESKTOP)
