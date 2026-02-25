// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#include "robotick/api.h"

namespace robotick
{
	struct QuatToEulerConfig
	{
		// Which computed angle feeds each output? 0: roll, 1: pitch, 2: yaw
		int output_roll_source = 0;
		int output_pitch_source = 1;
		int output_yaw_source = 2;
	};

	struct QuatToEulerInputs
	{
		Quatf quat;
	};

	struct QuatToEulerOutputs
	{
		float roll = 0.0f;	// radians
		float pitch = 0.0f; // radians
		float yaw = 0.0f;	// radians
	};

	struct QuatToEulerWorkload
	{
		QuatToEulerConfig config;
		QuatToEulerInputs inputs;
		QuatToEulerOutputs outputs;

		static inline int clamp_index(int index) { return robotick::clamp(index, 0, 2); }

		void tick(const TickInfo& info)
		{
			(void)info; // unused

			const Quatf quat_norm = inputs.quat.normalized();

			const float w = quat_norm.w;
			const float x = quat_norm.x;
			const float y = quat_norm.y;
			const float z = quat_norm.z;

			// Standard REP-103 convention
			// In Robotick orientation semantics, yaw is about +Z in a right-handed frame.
			const float sinr_cosp = 2.0f * (w * x + y * z);
			const float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
			const float roll = atan2f(sinr_cosp, cosr_cosp);

			float sinp = 2.0f * (w * y - z * x);
			sinp = robotick::clamp(sinp, -1.0f, 1.0f); // Clamp to handle gimbal lock at pitch = ±90°
			const float pitch = asinf(sinp);

			const float siny_cosp = 2.0f * (w * z + x * y);
			const float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
			const float yaw = atan2f(siny_cosp, cosy_cosp);

			// Remap outputs according to config
			const float euler_angles[3] = {roll, pitch, yaw};
			const int index_roll = clamp_index(config.output_roll_source);
			const int index_pitch = clamp_index(config.output_pitch_source);
			const int index_yaw = clamp_index(config.output_yaw_source);

			outputs.roll = euler_angles[index_roll];
			outputs.pitch = euler_angles[index_pitch];
			outputs.yaw = euler_angles[index_yaw];
		}
	};
} // namespace robotick
