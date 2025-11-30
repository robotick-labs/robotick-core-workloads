// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#pragma once

#if defined(ROBOTICK_PLATFORM_DESKTOP) || defined(ROBOTICK_PLATFORM_LINUX)

#include "robotick/framework/concurrency/Sync.h"
#include "robotick/framework/containers/HeapVector.h"
#include "robotick/framework/strings/FixedString.h"
#include "robotick/framework/utility/Function.h"

#include <cstdint>
#include <mqtt.h> // now available to tests because of PUBLIC link above

namespace robotick
{
	struct BrokerAddress
	{
		FixedString256 host;
		uint16_t port = 1883;
	};

	enum class MqttOpResult
	{
		Success,
		Dropped,
		Error,
	};

	class IMqttClient
	{
	  public:
		virtual ~IMqttClient() = default;
		virtual void connect() = 0;
		virtual MqttOpResult subscribe(const char* topic, int qos = 1) = 0;
		virtual MqttOpResult publish(const char* topic, const char* payload, bool retained = true) = 0;
		virtual void set_callback(Function<void(const char*, const char*)> on_message) = 0;
		virtual void set_tls_enabled(bool enabled) { (void)enabled; }
		virtual void set_qos(uint8_t publish_qos, uint8_t subscribe_qos)
		{
			(void)publish_qos;
			(void)subscribe_qos;
		}
	};

	class MqttClient : public IMqttClient
	{
	  public:
		MqttClient(const char* broker_uri, const char* client_id);
		~MqttClient() override; // declare

		void set_callback(Function<void(const char*, const char*)>) override;
		void connect() override;
		MqttOpResult subscribe(const char* topic, int qos = 1) override;
		MqttOpResult publish(const char* topic, const char* payload, bool retained = true) override;
		void set_tls_enabled(bool enabled) override;
		void set_qos(uint8_t publish_qos, uint8_t subscribe_qos) override;

		// Optional: drive mqtt-c from your engine tick
		void poll();	   // declare
		void disconnect(); // declare

		bool is_connected() const;
		struct HealthMetrics
		{
			uint32_t reconnect_attempts = 0;
			uint32_t consecutive_connect_failures = 0;
			uint32_t total_connect_failures = 0;
			uint32_t total_successful_connects = 0;
			uint64_t last_success_timestamp_ms = 0;

			bool healthy() const { return consecutive_connect_failures < 3; }
		};
		struct BackpressureStats
		{
			uint32_t publish_drops = 0;
			uint32_t subscribe_drops = 0;
			uint64_t last_drop_timestamp_ms = 0;
		};
		const HealthMetrics& get_health_metrics() const;
		const BackpressureStats& get_backpressure_stats() const;

	  private:
		// exact mqtt-c types
		struct mqtt_client mqtt;

		int sockfd = -1;
		Function<void(const char*, const char*)> message_callback;
		FixedString256 broker_host;
		uint16_t broker_port = 1883;
		FixedString128 client_id;
		HeapVector<uint8_t> sendbuf;
		HeapVector<uint8_t> recvbuf;
		FixedString256 inbound_topic;
		FixedString1024 inbound_payload;

		// static publish callback with access to private members
		static void on_publish(void** state, struct mqtt_response_publish* published);

		void initialize_buffers();
		bool assign_topic_payload(const mqtt_response_publish& published, FixedString256& topic_out, FixedString1024& payload_out);
		bool ensure_socket_timeout(int seconds);
		bool check_result(int rc, const char* tag);
		bool attempt_connect(bool fatal);
		void cleanup_socket();
		bool should_attempt_reconnect(uint64_t now) const;
		uint64_t now_ms() const;
		uint32_t compute_backoff_ms() const;
		void schedule_backoff(uint64_t now);
		bool ensure_connected_or_drop(bool publish);
		void record_backpressure(bool publish);

		Mutex operation_mutex;
		bool tls_enabled = false;
		uint8_t current_publish_qos = 0;
		uint8_t current_subscribe_qos = 0;
		bool mqtt_initialized = false;
		uint64_t next_connect_attempt_ms = 0;
		uint32_t base_backoff_ms = 500;
		uint32_t max_backoff_ms = 30000;
		HealthMetrics health_metrics;
		BackpressureStats backpressure_stats;
	};

} // namespace robotick

#endif // desktop/linux
