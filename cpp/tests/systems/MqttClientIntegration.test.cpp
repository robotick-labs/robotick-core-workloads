// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#include "robotick/systems/MqttClient.h"

#include "robotick/framework/concurrency/Sync.h"
#include "robotick/framework/strings/FixedString.h"

#include <catch2/catch_test_macros.hpp>

#include <csignal>
#include <cstdio>
#include <cstring>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

namespace robotick::tests
{
	namespace
	{
		constexpr char kEnvPython[] = ROBOTICK_CORE_ROOT "/.mqttenv/bin/python3";
		constexpr char kSystemPython[] = "/usr/bin/python3";
		constexpr char kBrokerScript[] = ROBOTICK_CORE_ROOT "/tools/mqtt_broker.py";
		constexpr char kTestTopic[] = "robotick/integration/topic";
		constexpr char kBrokerMessage[] = "welcome from broker";
		constexpr char kClientMessage[] = "client payload";
		constexpr uint16_t kBrokerPort = 1884;

		const char* select_python_interpreter()
		{
			if (::access(kEnvPython, X_OK) == 0)
				return kEnvPython;
			if (::access(kSystemPython, X_OK) == 0)
				return kSystemPython;
			return nullptr;
		}

		struct BrokerGuard
		{
			pid_t pid = -1;

			void stop()
			{
				if (pid > 0)
				{
					::kill(pid, SIGTERM);
					waitpid(pid, nullptr, 0);
					pid = -1;
				}
			}

			void reset(pid_t value)
			{
				if (pid == value)
					return;
				stop();
				pid = value;
			}

			~BrokerGuard() { stop(); }
		};

		class BrokerRuntime
		{
		  public:
			BrokerRuntime()
			{
				interpreter = select_python_interpreter();
				if (!interpreter)
					return;
				if (::access(kBrokerScript, R_OK) != 0)
				{
					interpreter = nullptr;
					return;
				}

				pid_t broker_pid = ::fork();
				if (broker_pid < 0)
				{
					interpreter = nullptr;
					return;
				}

				if (broker_pid == 0)
				{
					char port_arg[16];
					::snprintf(port_arg, sizeof(port_arg), "%u", kBrokerPort);
					::execlp(interpreter, interpreter, kBrokerScript, port_arg, static_cast<char*>(nullptr));
					::perror("execlp failed");
					::_exit(1);
				}

				guard.reset(broker_pid);
				usleep(500000);

				int status = 0;
				pid_t exited = ::waitpid(guard.pid, &status, WNOHANG);
				if (exited == guard.pid)
				{
					if (WIFEXITED(status))
					{
						::fprintf(stderr, "mqtt_broker.py exited early with status %d\n", WEXITSTATUS(status));
					}
					else if (WIFSIGNALED(status))
					{
						::fprintf(stderr, "mqtt_broker.py terminated by signal %d\n", WTERMSIG(status));
					}
					guard.stop();
				}
			}

			~BrokerRuntime() = default;

			bool available() const { return guard.pid > 0; }

		  private:
			const char* interpreter = nullptr;
			BrokerGuard guard{-1};
		};

		BrokerRuntime& get_broker_runtime()
		{
			static BrokerRuntime runtime;
			return runtime;
		}
	} // namespace

	TEST_CASE("MqttClient integration: broker publish/subscribe", "[mqtt-integration]")
	{
		auto& runtime = get_broker_runtime();
		INFO("Failed to start MQTT broker (requires Python + tools/mqtt_broker.py)");
		REQUIRE(runtime.available());

		char uri[64];
		::snprintf(uri, sizeof(uri), "mqtt://127.0.0.1:%u", kBrokerPort);

		MqttClient client(uri, "robotick-integration");
		client.set_socket_timeout_ms(200);
		Mutex callback_mutex;
		bool callback_called = false;
		FixedString128 received_topic;
		FixedString128 received_payload;

		client.set_callback(
			[&](const char* topic, const char* message)
			{
				LockGuard guard(callback_mutex);
				if (topic)
					received_topic.assign(topic, static_cast<size_t>(::strlen(topic)));
				if (message)
					received_payload.assign(message, static_cast<size_t>(::strlen(message)));
				callback_called = true;
			});

		bool connected = false;
		for (int attempt = 0; attempt < 100 && !connected; ++attempt)
		{
			if (client.connect())
			{
				connected = true;
			}
			else
			{
				usleep(50000);
			}
		}
		REQUIRE(connected);
		REQUIRE(client.subscribe(kTestTopic) == MqttOpResult::Success);

		for (int i = 0; i < 100; ++i)
		{
			client.poll();
			{
				LockGuard guard(callback_mutex);
				if (callback_called)
					break;
			}
			usleep(5000);
		}
		{
			LockGuard guard(callback_mutex);
			REQUIRE(callback_called);
		}
		CHECK(received_topic == kTestTopic);
		CHECK(received_payload == kBrokerMessage);

		REQUIRE(client.publish(kTestTopic, kClientMessage) == MqttOpResult::Success);

		client.disconnect();
	}
} // namespace robotick::tests
