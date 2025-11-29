// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#include "robotick/api.h"
#include "robotick/framework/containers/FixedVector.h"
#include "robotick/systems/Camera.h"
#include "robotick/systems/Image.h"

namespace robotick
{

	//------------------------------------------------------------------------------
	// Config / Inputs / Outputs
	//------------------------------------------------------------------------------

	struct CameraConfig
	{
		int camera_index = 0;
	};

	struct CameraInputs
	{
	};

	struct CameraOutputs
	{
		ImageJpeg128k jpeg_data;
	};

	//------------------------------------------------------------------------------
	// State (not currently needed, but here for expansion)
	//------------------------------------------------------------------------------

	struct CameraState
	{
		Camera camera;
	};

	struct CameraWorkload
	{
		CameraConfig config;
		CameraInputs inputs;
		CameraOutputs outputs;
		State<CameraState> state;

		void load()
		{
			if (!state->camera.setup(config.camera_index))
			{
				state->camera.print_available_cameras();
				ROBOTICK_FATAL_EXIT("CameraWorkload failed to initialize camera index %i", config.camera_index);
			}
		}

		void tick(const TickInfo& tick_info)
		{
			(void)tick_info;

			size_t size_used = 0;
			if (state->camera.read_frame(outputs.jpeg_data.data(), outputs.jpeg_data.capacity(), size_used))
			{
				outputs.jpeg_data.set_size(size_used);
			}
		}
	};

} // namespace robotick
