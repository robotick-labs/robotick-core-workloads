// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#include "robotick/api.h"
#include "robotick/platform/WebServer.h"
#include "robotick/systems/Image.h"

#if defined(ROBOTICK_PLATFORM_DESKTOP)
#include <nlohmann/json.hpp>
#endif // #if defined(ROBOTICK_PLATFORM_DESKTOP)

namespace robotick
{
	struct RemoteControlConfig
	{
		int port = 7080;
		FixedString128 web_root_folder = "engine-data/remote_control_interface_web";
	};

	struct RemoteControlOutputs
	{
		bool use_web_inputs = true;

		// Analog sticks (normalized -1..1)
		Vec2f left;
		Vec2f right;

		// Optional per-axis scaling applied in tick()
		Vec2f scale_left{1.0f, 1.0f};
		Vec2f scale_right{1.0f, 1.0f};

		// Triggers (normalized 0..1)
		float left_trigger = 0.0f;
		float right_trigger = 0.0f;

		// --- Added: Xbox 360 button booleans ---
		bool a = false;
		bool b = false;
		bool x = false;
		bool y = false;
		bool left_bumper = false;
		bool right_bumper = false;
		bool back = false;
		bool start = false;
		bool guide = false;
		bool left_stick_button = false;
		bool right_stick_button = false;
		bool dpad_up = false;
		bool dpad_down = false;
		bool dpad_left = false;
		bool dpad_right = false;
	};

	struct RemoteControlState
	{
		RemoteControlOutputs web_inputs;
		WebServer server;
	};

	struct RemoteControlWorkload
	{
		RemoteControlConfig config;
		RemoteControlOutputs outputs;

		State<RemoteControlState> state;

		void setup()
		{
#if defined(ROBOTICK_PLATFORM_DESKTOP)
			state->server.start("RemoteControl",
				config.port,
				config.web_root_folder.c_str(),
				[&](const WebRequest& request, WebResponse& response)
				{
					if (request.method == "POST" && request.uri == "/api/rc_state")
					{
						const auto json_opt = nlohmann::json::parse(request.body, nullptr, /*allow exceptions*/ false);
						if (json_opt.is_discarded())
						{
							response.set_status_code(WebResponseCode::BadRequest);
							response.set_body_string("Invalid JSON format.");
							return true; // handled
						}

						const nlohmann::json& json = json_opt;
						auto& web_inputs = state->web_inputs;

						// Helper setters
						auto try_set_vec2_from_json = [&](const char* name, Vec2f& out_vec2)
						{
							if (!json.contains(name))
								return;
							const auto& obj = json[name];
							if (!obj.is_object())
								return;

							if (obj.contains("x") && obj["x"].is_number())
								out_vec2.x = obj["x"].get<float>();
							if (obj.contains("y") && obj["y"].is_number())
								out_vec2.y = obj["y"].get<float>();
						};

						auto try_set_bool = [&](const char* name, bool& out_bool)
						{
							if (json.contains(name) && json[name].is_boolean())
								out_bool = json[name].get<bool>();
						};

						auto try_set_number = [&](const char* name, float& out_f32)
						{
							if (json.contains(name) && json[name].is_number())
								out_f32 = json[name].get<float>();
						};

						// Core fields
						try_set_bool("use_web_inputs", web_inputs.use_web_inputs);
						try_set_vec2_from_json("left", web_inputs.left);
						try_set_vec2_from_json("right", web_inputs.right);

						try_set_number("left_trigger", web_inputs.left_trigger);
						try_set_number("right_trigger", web_inputs.right_trigger);

						// --- Added: Xbox 360 button booleans ---
						try_set_bool("a", web_inputs.a);
						try_set_bool("b", web_inputs.b);
						try_set_bool("x", web_inputs.x);
						try_set_bool("y", web_inputs.y);
						try_set_bool("left_bumper", web_inputs.left_bumper);
						try_set_bool("right_bumper", web_inputs.right_bumper);
						try_set_bool("back", web_inputs.back);
						try_set_bool("start", web_inputs.start);
						try_set_bool("guide", web_inputs.guide);
						try_set_bool("left_stick_button", web_inputs.left_stick_button);
						try_set_bool("right_stick_button", web_inputs.right_stick_button);
						try_set_bool("dpad_up", web_inputs.dpad_up);
						try_set_bool("dpad_down", web_inputs.dpad_down);
						try_set_bool("dpad_left", web_inputs.dpad_left);
						try_set_bool("dpad_right", web_inputs.dpad_right);

						response.set_status_code(WebResponseCode::OK);
						return true; // handled
					}

					return false; // not handled
				});
#endif // #if defined(ROBOTICK_PLATFORM_DESKTOP)
		}

		void tick(const TickInfo&)
		{
			// simply copy web-requested inputs to outputs
			outputs = state->web_inputs.use_web_inputs ? state->web_inputs : RemoteControlOutputs{};
		}

		void stop() { state->server.stop(); }
	};

} // namespace robotick
