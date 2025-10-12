// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#include "robotick/api.h"
#include <cmath>

namespace robotick
{
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
		QuatToEulerInputs inputs;
		QuatToEulerOutputs outputs;

		static inline float rad2deg(float r) { return r * (180.0f / static_cast<float>(M_PI)); }

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
			float pitch = std::atan2(sinr_cosp, cosr_cosp);

			float sinp = 2.0f * (w * y - z * x);
			sinp = std::clamp(sinp, -1.0f, 1.0f);
			float roll = std::asin(sinp);

			const float siny_cosp = 2.0f * (w * z + x * y);
			const float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
			float yaw = std::atan2(siny_cosp, cosy_cosp);

			outputs.roll = roll;
			outputs.pitch = pitch;
			outputs.yaw = yaw;
		}
	};
} // namespace robotick
