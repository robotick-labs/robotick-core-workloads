// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#include "robotick/systems/MqttClient.h"

#include "robotick/api.h"

#include <arpa/inet.h>
#include <cstring>
#include <netdb.h>
#include <sys/socket.h>
#include <unistd.h>

namespace robotick
{
	namespace
	{
		bool starts_with(const char* text, const char* prefix)
		{
			if (!text || !prefix)
				return false;

			while (*prefix)
			{
				if (*text == '\0' || *text != *prefix)
					return false;
				++text;
				++prefix;
			}
			return true;
		}

		bool parse_port(const char* str, uint16_t& out_port)
		{
			if (!str || *str == '\0')
				return false;
			int value = 0;
			while (*str)
			{
				if (*str < '0' || *str > '9')
					return false;
				value = (value * 10) + (*str - '0');
				if (value > 65535)
					return false;
				++str;
			}
			out_port = static_cast<uint16_t>(value);
			return true;
		}

		bool parse_broker_uri(const char* uri, BrokerAddress& out)
		{
			if (!uri || uri[0] == '\0')
				return false;

			const char* cursor = uri;
			if (starts_with(uri, "mqtt://"))
			{
				cursor += 7; // strlen("mqtt://")
			}

			const char* colon = std::strchr(cursor, ':');
			if (colon)
			{
				out.host.assign(cursor, static_cast<size_t>(colon - cursor));
				if (!parse_port(colon + 1, out.port))
					return false;
			}
			else
			{
				out.host.assign(cursor, fixed_strlen(cursor));
			}

			return !out.host.empty();
		}

		bool copy_into(FixedString256& destination, const void* src, size_t size)
		{
			destination.assign(static_cast<const char*>(src), size);
			return destination.length() == size;
		}

		bool copy_into(FixedString1024& destination, const void* src, size_t size)
		{
			destination.assign(static_cast<const char*>(src), size);
			return destination.length() == size;
		}
	} // namespace

	void MqttClient::on_publish(void** state, struct mqtt_response_publish* published)
	{
		if (!state || !*state || !published)
			return;

		auto* self = static_cast<MqttClient*>(*state);
		if (!self->message_callback)
			return;

		if (!self->assign_topic_payload(*published, self->inbound_topic, self->inbound_payload))
			return;

		self->message_callback(self->inbound_topic.c_str(), self->inbound_payload.c_str());
	}

	MqttClient::MqttClient(const char* broker_uri, const char* client_id_in)
		: broker_port(1883)
	{
		BrokerAddress parsed;
		if (!parse_broker_uri(broker_uri, parsed))
		{
			ROBOTICK_FATAL_EXIT("MQTT: Invalid broker URI '%s'", broker_uri ? broker_uri : "(null)");
		}

		broker_host = parsed.host;
		broker_port = parsed.port;
		client_id.assign(client_id_in, client_id_in ? fixed_strlen(client_id_in) : 0);

		initialize_buffers();
	}

	MqttClient::~MqttClient()
	{
		disconnect();
	}

	void MqttClient::initialize_buffers()
	{
		if (sendbuf.size() == 0)
		{
			sendbuf.initialize(2048);
		}
		if (recvbuf.size() == 0)
		{
			recvbuf.initialize(2048);
		}
	}

	bool MqttClient::assign_topic_payload(const mqtt_response_publish& published, FixedString256& topic_out, FixedString1024& payload_out)
	{
		const bool topic_ok = copy_into(topic_out, published.topic_name, static_cast<size_t>(published.topic_name_size));
		const bool payload_ok =
			copy_into(payload_out, published.application_message, static_cast<size_t>(published.application_message_size));

		if (!topic_ok)
		{
			ROBOTICK_WARNING("MQTT: incoming topic truncated to %zu bytes", topic_out.length());
		}
		if (!payload_ok)
		{
			ROBOTICK_WARNING("MQTT: incoming payload truncated to %zu bytes", payload_out.length());
		}

		return topic_ok && payload_ok;
	}

	void MqttClient::set_callback(Function<void(const char*, const char*)> cb)
	{
		message_callback = std::move(cb);
	}

	void MqttClient::connect()
	{
		hostent* he = gethostbyname(broker_host.c_str());
		if (!he)
		{
			ROBOTICK_FATAL_EXIT("MQTT: DNS resolve failed for '%s'", broker_host.c_str());
		}

		sockfd = socket(AF_INET, SOCK_STREAM, 0);
		if (sockfd < 0)
		{
			ROBOTICK_FATAL_EXIT("MQTT: socket() failed");
		}

		sockaddr_in addr{};
		addr.sin_family = AF_INET;
		addr.sin_port = htons(broker_port);
		std::memcpy(&addr.sin_addr.s_addr, he->h_addr, static_cast<size_t>(he->h_length));

		if (::connect(sockfd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0)
		{
			::close(sockfd);
			sockfd = -1;
			ROBOTICK_FATAL_EXIT("MQTT: connect() to broker failed");
		}

		mqtt_init(&mqtt,
			static_cast<mqtt_pal_socket_handle>(sockfd),
			sendbuf.data(),
			sendbuf.size(),
			recvbuf.data(),
			recvbuf.size(),
			&MqttClient::on_publish);

		mqtt.publish_response_callback_state = this;

		const uint8_t connect_flags = 0;
		const uint16_t keep_alive = 400;

		mqtt_connect(&mqtt, client_id.c_str(), nullptr, nullptr, 0, nullptr, nullptr, connect_flags, keep_alive);
		mqtt_sync(&mqtt);
	}

	void MqttClient::disconnect()
	{
		if (sockfd >= 0)
		{
			mqtt_disconnect(&mqtt);
			mqtt_sync(&mqtt);
			::close(sockfd);
			sockfd = -1;
		}
	}

	void MqttClient::subscribe(const char* topic, int qos)
	{
		mqtt_subscribe(&mqtt, topic, static_cast<uint8_t>(qos));
		mqtt_sync(&mqtt);
	}

	void MqttClient::publish(const char* topic, const char* payload, bool /*retained*/)
	{
		const size_t payload_size = payload ? fixed_strlen(payload) : 0;
		mqtt_publish(&mqtt, topic, const_cast<char*>(payload), payload_size, MQTT_PUBLISH_QOS_0);
		mqtt_sync(&mqtt);
	}

	void MqttClient::poll()
	{
		mqtt_sync(&mqtt);
	}

} // namespace robotick
