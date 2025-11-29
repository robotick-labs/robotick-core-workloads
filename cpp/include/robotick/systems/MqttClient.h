// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "robotick/framework/containers/HeapVector.h"
#include "robotick/framework/strings/FixedString.h"
#include "robotick/framework/utility/Function.h"

#include <mqtt.h> // now available to tests because of PUBLIC link above

namespace robotick
{
	struct BrokerAddress
	{
		FixedString256 host;
		uint16_t port = 1883;
	};

	class IMqttClient
	{
	  public:
		virtual ~IMqttClient() = default;
		virtual void connect() = 0;
		virtual void subscribe(const char* topic, int qos = 1) = 0;
		virtual void publish(const char* topic, const char* payload, bool retained = true) = 0;
		virtual void set_callback(Function<void(const char*, const char*)> on_message) = 0;
	};

	class MqttClient : public IMqttClient
	{
	  public:
		MqttClient(const char* broker_uri, const char* client_id);
		~MqttClient() override; // declare

		void set_callback(Function<void(const char*, const char*)>) override;
		void connect() override;
		void subscribe(const char* topic, int qos = 1) override;
		void publish(const char* topic, const char* payload, bool retained = true) override;

		// Optional: drive mqtt-c from your engine tick
		void poll();	   // declare
		void disconnect(); // declare

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
	};

} // namespace robotick
