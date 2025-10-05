// MqttClient.cpp
#include "robotick/systems/MqttClient.h"

#include <cstring>
#include <netdb.h>
#include <netinet/in.h>
#include <stdexcept>
#include <sys/socket.h>
#include <unistd.h>

namespace robotick
{

	// ---- static (has access to private members) ----
	void MqttClient::on_publish(void** state, struct mqtt_response_publish* published)
	{
		if (!state || !*state || !published)
			return;
		auto* self = static_cast<MqttClient*>(*state);

		// mqtt-c uses void* + size
		std::string topic(static_cast<const char*>(published->topic_name), static_cast<size_t>(published->topic_name_size));
		std::string payload(static_cast<const char*>(published->application_message), static_cast<size_t>(published->application_message_size));

		if (self->message_callback)
		{
			self->message_callback(topic, payload);
		}
	}

	MqttClient::MqttClient(const std::string& broker_uri, const std::string& client_id_in)
		: broker_host("localhost")
		, broker_port(1883)
		, client_id(client_id_in)
		, sendbuf(2048)
		, recvbuf(2048)
	{
		// Parse mqtt://host:port | host:port | host
		std::string s = broker_uri;
		const std::string prefix = "mqtt://";
		if (s.rfind(prefix, 0) == 0)
			s = s.substr(prefix.size());

		auto pos = s.find(':');
		if (pos != std::string::npos)
		{
			broker_host = s.substr(0, pos);
			broker_port = std::stoi(s.substr(pos + 1));
		}
		else
		{
			broker_host = s;
		}
	}

	MqttClient::~MqttClient()
	{
		try
		{
			disconnect();
		}
		catch (...)
		{
		}
	}

	void MqttClient::connect()
	{
		// resolve
		hostent* he = gethostbyname(broker_host.c_str());
		if (!he)
			throw std::runtime_error("MQTT: DNS resolve failed for '" + broker_host + "'");

		// socket
		sockfd = socket(AF_INET, SOCK_STREAM, 0);
		if (sockfd < 0)
			throw std::runtime_error("MQTT: socket() failed");

		sockaddr_in addr{};
		addr.sin_family = AF_INET;
		addr.sin_port = htons(static_cast<uint16_t>(broker_port));
		std::memcpy(&addr.sin_addr.s_addr, he->h_addr, static_cast<size_t>(he->h_length));

		if (::connect(sockfd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0)
		{
			::close(sockfd);
			sockfd = -1;
			throw std::runtime_error("MQTT: connect() to broker failed");
		}

		// init with callback
		mqtt_init(&mqtt,
			static_cast<mqtt_pal_socket_handle>(sockfd),
			sendbuf.data(),
			sendbuf.size(),
			recvbuf.data(),
			recvbuf.size(),
			&MqttClient::on_publish);

		// pass 'this' back via callback state
		mqtt.publish_response_callback_state = this;

		// mqtt_connect(mqtt_client*, client_id, will_topic, will_msg, will_msg_size,
		//              username, password, connect_flags, keep_alive)
		const uint8_t connect_flags = 0; // no username/password, no will
		const uint16_t keep_alive = 400;

		mqtt_connect(&mqtt, client_id.c_str(), nullptr, nullptr, 0, nullptr, nullptr, connect_flags, keep_alive);

		// send CONNECT + read CONNACK
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

	void MqttClient::subscribe(const std::string& topic, int qos)
	{
		mqtt_subscribe(&mqtt, topic.c_str(), static_cast<uint8_t>(qos));
		mqtt_sync(&mqtt);
	}

	void MqttClient::publish(const std::string& topic, const std::string& payload, bool /*retained*/)
	{
		mqtt_publish(&mqtt, topic.c_str(), const_cast<char*>(payload.data()), payload.size(), MQTT_PUBLISH_QOS_0);
		mqtt_sync(&mqtt);
	}

	void MqttClient::set_callback(std::function<void(const std::string&, const std::string&)> cb)
	{
		message_callback = std::move(cb);
	}

	void MqttClient::poll()
	{
		// call from your engine tick (or a timer) to process IO
		mqtt_sync(&mqtt);
	}

} // namespace robotick
