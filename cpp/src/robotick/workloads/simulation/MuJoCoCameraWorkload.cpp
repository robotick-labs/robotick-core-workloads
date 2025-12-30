// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#if defined(ROBOTICK_PLATFORM_DESKTOP) || defined(ROBOTICK_PLATFORM_LINUX)

#include "robotick/api.h"
#include "robotick/framework/strings/FixedString.h"
#include "robotick/systems/Image.h"

namespace robotick
{
	// Stub workload: render a single MuJoCo camera (implementation later).

	struct MuJoCoCameraConfig
	{
		FixedString64 camera_name;
		uint32_t texture_width = 640;
		uint32_t texture_height = 480;
	};

	struct MuJoCoCameraInputs
	{
		uint32_t mujoco_scene_id = 0;
	};

	struct MuJoCoCameraOutputs
	{
		ImagePng128k png_data;
	};

	struct MuJoCoCameraWorkload
	{
		MuJoCoCameraConfig config;
		MuJoCoCameraInputs inputs;
		MuJoCoCameraOutputs outputs;

		void pre_load()
		{
		}

		void tick(const TickInfo&)
		{
			// TODO: fetch snapshot via MuJoCoSceneRegistry using inputs.scene_id,
			// render with mjv/mjr, and populate outputs.png.
			outputs.png_data.set_size(0);
		}
	};
} // namespace robotick

#endif // ROBOTICK_PLATFORM_DESKTOP || ROBOTICK_PLATFORM_LINUX
