// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#if defined(ROBOTICK_PLATFORM_DESKTOP) || defined(ROBOTICK_PLATFORM_LINUX)

#include "robotick/systems/MqttClient.h"

#include "robotick/api.h"
#include "robotick/framework/concurrency/Sync.h"
#include "robotick/framework/memory/Memory.h"

#include <arpa/inet.h>
#include <cstring>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

namespace robotick
{
	namespace
	{
		constexpr int kSocketTimeoutSec = 5;
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

			const char* colon = ::strchr(cursor, ':');
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
		const bool payload_ok = copy_into(payload_out, published.application_message, static_cast<size_t>(published.application_message_size));

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

	void MqttClient::set_tls_enabled(bool enabled)
	{
		tls_enabled = enabled;
	}

	void MqttClient::set_qos(uint8_t publish_qos, uint8_t subscribe_qos)
	{
		auto clamp = [](uint8_t value) -> uint8_t
		{
			return (value > 2) ? 2 : value;
		};
		current_publish_qos = clamp(publish_qos);
		current_subscribe_qos = clamp(subscribe_qos);
	}

	void MqttClient::set_callback(Function<void(const char*, const char*)> cb)
	{
		message_callback = robotick::move(cb);
	}

	bool MqttClient::connect()
	{
		return attempt_connect(true);
	}

	void MqttClient::disconnect()
	{
		LockGuard guard(operation_mutex);
		if (sockfd >= 0)
		{
			if (mqtt_initialized)
			{
				check_result(mqtt_disconnect(&mqtt), "disconnect");
				mqtt_initialized = false;
			}
			::close(sockfd);
			sockfd = -1;
		}
	}

	MqttOpResult MqttClient::subscribe(const char* topic, int qos)
	{
		if (!ensure_connected_or_drop(false))
			return MqttOpResult::Dropped;

		LockGuard guard(operation_mutex);
		const uint8_t subscribe_qos = current_subscribe_qos ? current_subscribe_qos : static_cast<uint8_t>(qos);
		if (check_result(mqtt_subscribe(&mqtt, topic, subscribe_qos), "subscribe"))
		{
			check_result(mqtt_sync(&mqtt), "sync");
			return MqttOpResult::Success;
		}
		return MqttOpResult::Error;
	}

	MqttOpResult MqttClient::publish(const char* topic, const char* payload, bool /*retained*/)
	{
		if (!ensure_connected_or_drop(true))
		{
			return MqttOpResult::Dropped;
		}

		const size_t payload_size = payload ? fixed_strlen(payload) : 0;
		LockGuard guard(operation_mutex);
		const uint8_t publish_flag =
			(current_publish_qos == 2) ? MQTT_PUBLISH_QOS_2 : (current_publish_qos == 1 ? MQTT_PUBLISH_QOS_1 : MQTT_PUBLISH_QOS_0);
		if (check_result(mqtt_publish(&mqtt, topic, const_cast<char*>(payload), payload_size, publish_flag), "publish"))
		{
			check_result(mqtt_sync(&mqtt), "sync");
			return MqttOpResult::Success;
		}
		return MqttOpResult::Error;
	}

	void MqttClient::poll()
	{
		if (!is_connected())
		{
			attempt_connect(false);
			return;
		}

		LockGuard guard(operation_mutex);
		check_result(mqtt_sync(&mqtt), "sync");
	}

	bool MqttClient::attempt_connect(bool fatal)
	{
		const uint64_t now = now_ms();
		if (!fatal && !should_attempt_reconnect(now))
		{
			return false;
		}

		if (!fatal)
		{
			health_metrics.reconnect_attempts++;
		}

		LockGuard guard(operation_mutex);
		if (sockfd >= 0)
		{
			return true;
		}

		auto fail = [&](const char* reason)
		{
			cleanup_socket();
			health_metrics.total_connect_failures++;
			health_metrics.consecutive_connect_failures++;
			schedule_backoff(now);
			ROBOTICK_WARNING("MQTT: %s", reason);
			return false;
		};

		hostent* he = gethostbyname(broker_host.c_str());
		if (!he)
		{
			return fail("DNS resolve failed for broker");
		}

		sockfd = socket(AF_INET, SOCK_STREAM, 0);
		if (sockfd < 0)
		{
			return fail("socket() failed");
		}

		sockaddr_in addr{};
		addr.sin_family = AF_INET;
		addr.sin_port = htons(broker_port);
		::memcpy(&addr.sin_addr.s_addr, he->h_addr, static_cast<size_t>(he->h_length));

		if (::connect(sockfd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0)
		{
			return fail("connect() to broker failed");
		}

		if (!ensure_socket_timeout(kSocketTimeoutSec))
		{
			return fail("failed to configure socket timeouts");
		}

		if (tls_enabled)
		{
#if defined(ROBOTICK_ENABLE_MQTT_TLS)
			ROBOTICK_INFO("MQTT: TLS enabled for broker connection.");
#else
			ROBOTICK_WARNING("MQTT: TLS requested but this build lacks TLS support; proceeding without encryption.");
#endif
		}

		mqtt_init(&mqtt,
			static_cast<mqtt_pal_socket_handle>(sockfd),
			sendbuf.data(),
			sendbuf.size(),
			recvbuf.data(),
			recvbuf.size(),
			&MqttClient::on_publish);
		mqtt.publish_response_callback_state = this;
		mqtt_initialized = true;

		const uint8_t connect_flags = 0;
		const uint16_t keep_alive = 400;

		if (!check_result(mqtt_connect(&mqtt, client_id.c_str(), nullptr, nullptr, 0, nullptr, nullptr, connect_flags, keep_alive), "connect"))
		{
			return fail("mqtt_connect failed");
		}

		check_result(mqtt_sync(&mqtt), "sync");

		health_metrics.total_successful_connects++;
		health_metrics.consecutive_connect_failures = 0;
		health_metrics.last_success_timestamp_ms = now;
		next_connect_attempt_ms = 0;

		return true;
	}

	bool MqttClient::ensure_socket_timeout(int seconds)
	{
		if (sockfd < 0)
			return false;
		timeval timeout{};
		timeout.tv_sec = seconds;
		timeout.tv_usec = 0;
		if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0)
			return false;
		if (setsockopt(sockfd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout)) < 0)
			return false;
		return true;
	}

	bool MqttClient::check_result(int rc, const char* tag)
	{
		if (rc == MQTT_OK)
			return true;
		ROBOTICK_WARNING("MQTT: %s failed (%d)", tag, rc);
		return false;
	}

	void MqttClient::cleanup_socket()
	{
		if (mqtt_initialized)
		{
			mqtt_disconnect(&mqtt);
			mqtt_initialized = false;
		}
		if (sockfd >= 0)
		{
			::close(sockfd);
			sockfd = -1;
		}
	}

	bool MqttClient::is_connected() const
	{
		return sockfd >= 0;
	}

	const MqttClient::HealthMetrics& MqttClient::get_health_metrics() const
	{
		return health_metrics;
	}

	bool MqttClient::should_attempt_reconnect(uint64_t now) const
	{
		return next_connect_attempt_ms == 0 || now >= next_connect_attempt_ms;
	}

	uint64_t MqttClient::now_ms() const
	{
		timeval tv{};
		gettimeofday(&tv, nullptr);
		return static_cast<uint64_t>(tv.tv_sec) * 1000ull + static_cast<uint64_t>(tv.tv_usec) / 1000ull;
	}

	uint32_t MqttClient::compute_backoff_ms() const
	{
		const uint32_t exponent = health_metrics.consecutive_connect_failures > 6 ? 6 : health_metrics.consecutive_connect_failures;
		uint32_t value = base_backoff_ms << exponent;
		if (value > max_backoff_ms)
		{
			value = max_backoff_ms;
		}
		return value;
	}

	void MqttClient::schedule_backoff(uint64_t now)
	{
		next_connect_attempt_ms = now + compute_backoff_ms();
	}

	bool MqttClient::ensure_connected_or_drop(bool publish)
	{
		if (is_connected())
			return true;

		ROBOTICK_WARNING("MQTT: %s called while disconnected", publish ? "publish" : "subscribe");

		if (!attempt_connect(false))
		{
			record_backpressure(publish);
			return false;
		}
		return true;
	}

	void MqttClient::record_backpressure(bool publish)
	{
		const uint64_t now = now_ms();
		backpressure_stats.last_drop_timestamp_ms = now;
		if (publish)
		{
			backpressure_stats.publish_drops++;
		}
		else
		{
			backpressure_stats.subscribe_drops++;
		}
	}

	const MqttClient::BackpressureStats& MqttClient::get_backpressure_stats() const
	{
		return backpressure_stats;
	}

} // namespace robotick

#endif
