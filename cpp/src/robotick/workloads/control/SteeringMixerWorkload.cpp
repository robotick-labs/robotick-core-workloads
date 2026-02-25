// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#include "robotick/api.h"

#include <algorithm>

namespace robotick
{

	// === Field registrations ===

	struct SteeringMixerConfig
	{
		float max_speed_differential = 0.4f;
		float power_scale_both = 1.0f;
		float power_scale_left = 1.0f;
		float power_scale_right = 1.0f;
		float power_seek_rate = -1.0f; // <= 0 means instant-snap (no seeking)
	};

	struct SteeringMixerInputs
	{
		float speed = 0.0f;
		float angular_speed = 0.0f;
	};

	struct SteeringMixerOutputs
	{
		float left_motor = 0.0f;
		float right_motor = 0.0f;
	};

	// === Workload ===

	struct SteeringMixerWorkload
	{
		SteeringMixerInputs inputs;
		SteeringMixerOutputs outputs;
		SteeringMixerConfig config;

		void tick(const TickInfo& tick_info)
		{
			const float speed = inputs.speed;
			const float turn = inputs.angular_speed;

			// Right-handed Z-up yaw convention:
			// positive angular_speed => positive yaw about +Z => CCW/top-view => left turn.
			// For a differential drive, a left turn means right motor > left motor.
			// `speed` preserves the robot's existing model-level forward-axis choice.
			float left = speed - turn * config.max_speed_differential;
			float right = speed + turn * config.max_speed_differential;

			// Clamp to [-1, 1]
			left = max(-1.0f, min(1.0f, left));
			right = max(-1.0f, min(1.0f, right));

			left *= config.power_scale_both * config.power_scale_left;
			right *= config.power_scale_both * config.power_scale_right;

			if (config.power_seek_rate <= 0.0f)
			{
				outputs.left_motor = left;
				outputs.right_motor = right;
			}
			else
			{
				const float max_delta = config.power_seek_rate * tick_info.delta_time;
				const auto seek_towards = [max_delta](float current, float target)
				{
					const float delta = target - current;
					if (delta > max_delta)
					{
						return current + max_delta;
					}
					if (delta < -max_delta)
					{
						return current - max_delta;
					}
					return target;
				};

				outputs.left_motor = seek_towards(outputs.left_motor, left);
				outputs.right_motor = seek_towards(outputs.right_motor, right);
			}
		}
	};

} // namespace robotick
