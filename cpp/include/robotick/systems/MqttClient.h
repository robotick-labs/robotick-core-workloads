// MqttClient.h (only the relevant part)

#pragma once
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <mqtt.h> // now available to tests because of PUBLIC link above

namespace robotick
{

	class IMqttClient
	{
	  public:
		virtual ~IMqttClient() = default;
		virtual void connect() = 0;
		virtual void subscribe(const std::string& topic, int qos = 1) = 0;
		virtual void publish(const std::string& topic, const std::string& payload, bool retained = true) = 0;
		virtual void set_callback(std::function<void(const std::string&, const std::string&)> on_message) = 0;
	};

	class MqttClient : public IMqttClient
	{
	  public:
		MqttClient(const std::string& broker_uri, const std::string& client_id);
		~MqttClient() override; // ✅ declare

		void set_callback(std::function<void(const std::string&, const std::string&)>) override;
		void connect() override;
		void subscribe(const std::string& topic, int qos = 1) override;
		void publish(const std::string& topic, const std::string& payload, bool retained = true) override;

		// Optional: drive mqtt-c from your engine tick
		void poll();	   // ✅ declare
		void disconnect(); // ✅ declare

	  private:
		// exact mqtt-c types
		struct mqtt_client mqtt;

		int sockfd = -1;
		std::function<void(const std::string&, const std::string&)> message_callback;
		std::string broker_host;
		int broker_port = 1883;
		std::string client_id;
		std::vector<uint8_t> sendbuf;
		std::vector<uint8_t> recvbuf;

		// static publish callback with access to private members
		static void on_publish(void** state, struct mqtt_response_publish* published);
	};

} // namespace robotick
