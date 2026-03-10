// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#include "robotick/api.h"

namespace robotick
{
	enum class StickShapeTransform : uint8_t
	{
		None = 0,
		CircleToSquare
	};

	ROBOTICK_REGISTER_ENUM_BEGIN(StickShapeTransform)
	ROBOTICK_ENUM_VALUE("None", StickShapeTransform::None)
	ROBOTICK_ENUM_VALUE("CircleToSquare", StickShapeTransform::CircleToSquare)
	ROBOTICK_REGISTER_ENUM_END(StickShapeTransform)

	struct RemoteControlConfig
	{
		Vec2f dead_zone_left = Vec2f{0.1f, 0.1f};
		Vec2f dead_zone_right = Vec2f{0.1f, 0.1f};
		StickShapeTransform stick_shape_transform_left = StickShapeTransform::CircleToSquare;
		StickShapeTransform stick_shape_transform_right = StickShapeTransform::CircleToSquare;
	};

	struct GamepadState
	{
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

	struct RemoteControlInputs
	{
		bool use_web_inputs = true;
		GamepadState gamepad_state_raw;
	};

	struct RemoteControlOutputs
	{
		GamepadState gamepad_state;
	};

	struct RemoteControlWorkload
	{
		RemoteControlConfig config;
		RemoteControlInputs inputs;
		RemoteControlOutputs outputs;

		static float apply_dead_zone(float value, float dead_zone)
		{
			const float clamped_dead_zone = robotick::clamp(dead_zone, 0.0f, 0.99f);
			if (robotick::abs(value) < clamped_dead_zone)
			{
				return 0.0f;
			}

			const float direction = value >= 0.0f ? 1.0f : -1.0f;
			return (value - (direction * clamped_dead_zone)) / (1.0f - clamped_dead_zone);
		}

		static Vec2f apply_circle_to_square(const Vec2f& input)
		{
			const float radius = robotick::sqrt((input.x * input.x) + (input.y * input.y));
			if (radius <= 1e-6f)
			{
				return Vec2f{0.0f, 0.0f};
			}

			const float max_axis = robotick::max(robotick::abs(input.x), robotick::abs(input.y));
			const float scale = radius / max_axis;
			return Vec2f{robotick::clamp(input.x * scale, -1.0f, 1.0f), robotick::clamp(input.y * scale, -1.0f, 1.0f)};
		}

		static Vec2f apply_stick_shape_transform(const Vec2f& input, StickShapeTransform transform)
		{
			switch (transform)
			{
			case StickShapeTransform::CircleToSquare:
				return apply_circle_to_square(input);
			case StickShapeTransform::None:
			default:
				return input;
			}
		}

		void tick(const TickInfo&)
		{
			// Copy requested gamepad state to outputs, then apply authoritative transforms/dead-zones/scales.
			outputs.gamepad_state = inputs.use_web_inputs ? inputs.gamepad_state_raw : GamepadState{};

			outputs.gamepad_state.left = apply_stick_shape_transform(outputs.gamepad_state.left, config.stick_shape_transform_left);
			outputs.gamepad_state.right = apply_stick_shape_transform(outputs.gamepad_state.right, config.stick_shape_transform_right);

			// apply dead-zones to each stick:
			outputs.gamepad_state.left.x = apply_dead_zone(outputs.gamepad_state.left.x, config.dead_zone_left.x);
			outputs.gamepad_state.left.y = apply_dead_zone(outputs.gamepad_state.left.y, config.dead_zone_left.y);
			outputs.gamepad_state.right.x = apply_dead_zone(outputs.gamepad_state.right.x, config.dead_zone_right.x);
			outputs.gamepad_state.right.y = apply_dead_zone(outputs.gamepad_state.right.y, config.dead_zone_right.y);

			// Optional per-axis scaling applied after dead-zones.
			outputs.gamepad_state.left.x = robotick::clamp(outputs.gamepad_state.left.x * outputs.gamepad_state.scale_left.x, -1.0f, 1.0f);
			outputs.gamepad_state.left.y = robotick::clamp(outputs.gamepad_state.left.y * outputs.gamepad_state.scale_left.y, -1.0f, 1.0f);
			outputs.gamepad_state.right.x = robotick::clamp(outputs.gamepad_state.right.x * outputs.gamepad_state.scale_right.x, -1.0f, 1.0f);
			outputs.gamepad_state.right.y = robotick::clamp(outputs.gamepad_state.right.y * outputs.gamepad_state.scale_right.y, -1.0f, 1.0f);
		}
	};

	ROBOTICK_REGISTER_STRUCT_BEGIN(GamepadState)
	ROBOTICK_STRUCT_FIELD(GamepadState, Vec2f, left)
	ROBOTICK_STRUCT_FIELD(GamepadState, Vec2f, right)
	ROBOTICK_STRUCT_FIELD(GamepadState, Vec2f, scale_left)
	ROBOTICK_STRUCT_FIELD(GamepadState, Vec2f, scale_right)
	ROBOTICK_STRUCT_FIELD(GamepadState, float, left_trigger)
	ROBOTICK_STRUCT_FIELD(GamepadState, float, right_trigger)
	ROBOTICK_STRUCT_FIELD(GamepadState, bool, a)
	ROBOTICK_STRUCT_FIELD(GamepadState, bool, b)
	ROBOTICK_STRUCT_FIELD(GamepadState, bool, x)
	ROBOTICK_STRUCT_FIELD(GamepadState, bool, y)
	ROBOTICK_STRUCT_FIELD(GamepadState, bool, left_bumper)
	ROBOTICK_STRUCT_FIELD(GamepadState, bool, right_bumper)
	ROBOTICK_STRUCT_FIELD(GamepadState, bool, back)
	ROBOTICK_STRUCT_FIELD(GamepadState, bool, start)
	ROBOTICK_STRUCT_FIELD(GamepadState, bool, guide)
	ROBOTICK_STRUCT_FIELD(GamepadState, bool, left_stick_button)
	ROBOTICK_STRUCT_FIELD(GamepadState, bool, right_stick_button)
	ROBOTICK_STRUCT_FIELD(GamepadState, bool, dpad_up)
	ROBOTICK_STRUCT_FIELD(GamepadState, bool, dpad_down)
	ROBOTICK_STRUCT_FIELD(GamepadState, bool, dpad_left)
	ROBOTICK_STRUCT_FIELD(GamepadState, bool, dpad_right)
	ROBOTICK_REGISTER_STRUCT_END(GamepadState)

} // namespace robotick
