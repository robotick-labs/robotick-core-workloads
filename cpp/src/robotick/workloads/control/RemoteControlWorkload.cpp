// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#include "robotick/api.h"
#include "robotick/platform/WebServer.h"

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

	struct RemoteControlInputs
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

		// JPEG passthrough
		FixedVector128k jpeg_data;

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

	struct RemoteControlOutputs
	{
		// Mirrored/processed controls
		Vec2f left;
		Vec2f right;

		float left_trigger = 0.0f;
		float right_trigger = 0.0f;

		// --- Added: mirror button booleans to outputs ---
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
		WebServer server;
		RemoteControlInputs web_inputs;
	};

	struct RemoteControlWorkload
	{
		RemoteControlConfig config;
		RemoteControlInputs inputs;
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
					if (request.method == "POST" && request.uri == "/api/joystick_input")
					{
						const auto json_opt = nlohmann::json::parse(request.body, nullptr, /*allow exceptions*/ false);
						if (json_opt.is_discarded())
						{
							response.status_code = 400;
							response.body.set_from_string("Invalid JSON format.");
							return true; // handled
						}

						const nlohmann::json& json = json_opt;
						auto& w = state->web_inputs;

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
						try_set_bool("use_web_inputs", w.use_web_inputs);
						try_set_vec2_from_json("left", w.left);
						try_set_vec2_from_json("right", w.right);

						try_set_number("left_trigger", w.left_trigger);
						try_set_number("right_trigger", w.right_trigger);

						// --- Added: Xbox 360 button booleans ---
						try_set_bool("a", w.a);
						try_set_bool("b", w.b);
						try_set_bool("x", w.x);
						try_set_bool("y", w.y);
						try_set_bool("left_bumper", w.left_bumper);
						try_set_bool("right_bumper", w.right_bumper);
						try_set_bool("back", w.back);
						try_set_bool("start", w.start);
						try_set_bool("guide", w.guide);
						try_set_bool("left_stick_button", w.left_stick_button);
						try_set_bool("right_stick_button", w.right_stick_button);
						try_set_bool("dpad_up", w.dpad_up);
						try_set_bool("dpad_down", w.dpad_down);
						try_set_bool("dpad_left", w.dpad_left);
						try_set_bool("dpad_right", w.dpad_right);

						response.status_code = 200;
						return true; // handled
					}
					else if (request.method == "GET" && request.uri == "/api/jpeg_data")
					{
						response.body.set(inputs.jpeg_data.data(), inputs.jpeg_data.size());
						response.content_type = "image/jpeg";
						response.status_code = 200;
						return true; // handled
					}

					return false; // not handled
				});
#endif // #if defined(ROBOTICK_PLATFORM_DESKTOP)
		}

		void tick(const TickInfo&)
		{
			// Honour web input takeover if either side requests it
			const bool use_web_inputs_flag = inputs.use_web_inputs || state->web_inputs.use_web_inputs;
			const RemoteControlInputs& in_ref = use_web_inputs_flag ? state->web_inputs : inputs;

			// Sticks with scaling
			outputs.left.x = in_ref.left.x * in_ref.scale_left.x;
			outputs.left.y = in_ref.left.y * in_ref.scale_left.y;
			outputs.right.x = in_ref.right.x * in_ref.scale_right.x;
			outputs.right.y = in_ref.right.y * in_ref.scale_right.y;

			// Triggers
			outputs.left_trigger = in_ref.left_trigger;
			outputs.right_trigger = in_ref.right_trigger;

			// --- Mirror buttons to outputs ---
			outputs.a = in_ref.a;
			outputs.b = in_ref.b;
			outputs.x = in_ref.x;
			outputs.y = in_ref.y;
			outputs.left_bumper = in_ref.left_bumper;
			outputs.right_bumper = in_ref.right_bumper;
			outputs.back = in_ref.back;
			outputs.start = in_ref.start;
			outputs.guide = in_ref.guide;
			outputs.left_stick_button = in_ref.left_stick_button;
			outputs.right_stick_button = in_ref.right_stick_button;
			outputs.dpad_up = in_ref.dpad_up;
			outputs.dpad_down = in_ref.dpad_down;
			outputs.dpad_left = in_ref.dpad_left;
			outputs.dpad_right = in_ref.dpad_right;
		}

		void stop() { state->server.stop(); }

		static float apply_dead_zone(float value, float dead_zone)
		{
			if (std::abs(value) < dead_zone)
				return 0.0f;

			const float sign = (value > 0.0f) ? 1.0f : -1.0f;
			return ((std::abs(value) - dead_zone) / (1.0f - dead_zone)) * sign;
		}
	};

} // namespace robotick
