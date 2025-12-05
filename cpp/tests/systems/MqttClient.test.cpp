// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#include "robotick/systems/MqttClient.h"

#include <catch2/catch_test_macros.hpp>

#include <cstring>
#include <sys/socket.h>
#include <unistd.h>

namespace robotick::tests
{
	TEST_CASE("MQTT detail broker URI parsing", "[mqtt-detail]")
	{
		BrokerAddress parsed{};
		REQUIRE(mqtt_detail::parse_broker_uri("mqtt://example.com:8883", parsed));
		CHECK(::strcmp(parsed.host.c_str(), "example.com") == 0);
		CHECK(parsed.port == 8883);

		BrokerAddress default_port{};
		default_port.port = 1883;
		REQUIRE(mqtt_detail::parse_broker_uri("example.org", default_port));
		CHECK(::strcmp(default_port.host.c_str(), "example.org") == 0);
		CHECK(default_port.port == 1883);

		CHECK_FALSE(mqtt_detail::parse_broker_uri("", parsed));
		CHECK_FALSE(mqtt_detail::parse_broker_uri("mqtt://host:notaport", parsed));
	}

	TEST_CASE("MQTT detail sets socket timeouts", "[mqtt-detail]")
	{
		const int fd = socket(AF_INET, SOCK_STREAM, 0);
		REQUIRE(fd >= 0);
		CHECK(mqtt_detail::set_socket_timeout(fd, 1));
		close(fd);

		CHECK_FALSE(mqtt_detail::set_socket_timeout(-1, 1));
	}

	TEST_CASE("MqttClient QoS clamping", "[mqtt]")
	{
		MqttClient client("mqtt://localhost:1883", "test-qos");

#if defined(ROBOTICK_TEST_MODE)
		client.set_qos(5, 4);
		CHECK(client.get_publish_qos_for_test() == 2);
		CHECK(client.get_subscribe_qos_for_test() == 2);

		client.set_qos(1, 0);
		CHECK(client.get_publish_qos_for_test() == 1);
		CHECK(client.get_subscribe_qos_for_test() == 0);
#else
		SUCCEED("QoS helpers only available in test builds");
#endif
	}
} // namespace robotick::tests
