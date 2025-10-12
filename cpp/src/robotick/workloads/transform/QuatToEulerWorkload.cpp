// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#include "robotick/api.h"

#include <cmath>

namespace robotick
{
	struct QuatToEulerConfig
	{
		// Which computed angle feeds each output? 0: roll, 1: pitch, 2: yaw
		int roll_from = 0;
		int pitch_from = 1;
		int yaw_from = 2;
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

		static inline int clamp_index(int i) { return (i < 0) ? 0 : (i > 2 ? 2 : i); }

		void tick(const TickInfo& info)
		{
			(void)info; // unused

			const float w = inputs.quat.w;
			const float x = inputs.quat.x;
			const float y = inputs.quat.y;
			const float z = inputs.quat.z;

			// Standard aerospace convention (YXZ intrinsic)
			const float sinr_cosp = 2.0f * (w * x + y * z);
			const float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
			float roll = std::atan2(sinr_cosp, cosr_cosp);

			float sinp = 2.0f * (w * y - z * x);
			sinp = std::clamp(sinp, -1.0f, 1.0f);
			float pitch = std::asin(sinp);

			const float siny_cosp = 2.0f * (w * z + x * y);
			const float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
			float yaw = std::atan2(siny_cosp, cosy_cosp);

			// Remap outputs according to config
			const float euler_angles[3] = {roll, pitch, yaw};
			const int index_roll = clamp_index(config.roll_from);
			const int index_pitch = clamp_index(config.pitch_from);
			const int index_yaw = clamp_index(config.yaw_from);

			outputs.roll = euler_angles[index_roll];
			outputs.pitch = euler_angles[index_pitch];
			outputs.yaw = euler_angles[index_yaw];
		}
	};
} // namespace robotick
