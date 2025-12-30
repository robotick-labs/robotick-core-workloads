// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#if defined(ROBOTICK_PLATFORM_DESKTOP) || defined(ROBOTICK_PLATFORM_LINUX)

#include "robotick/api.h"
#include "robotick/framework/strings/FixedString.h"
#include "robotick/systems/Image.h"
#include "robotick/systems/MuJoCoRenderContext.h"
#include "robotick/systems/MuJoCoSceneRegistry.h"

#include <mujoco/mujoco.h>

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

	struct MuJoCoCameraState
	{
		// Render context owns the GL/SDL state for this camera workload.
		MuJoCoRenderContext render_context;
		bool render_context_ready = false;
		// Once disabled, this workload produces empty output without reallocating.
		bool render_disabled = false;
		// Model pointer used to ensure the mjData buffer matches the scene layout.
		const mjModel* render_model = nullptr;
		// Pre-allocated mjData buffer used for thread-safe snapshot copies.
		mjData* render_data = nullptr;
	};

	struct MuJoCoCameraWorkload
	{
		MuJoCoCameraConfig config;
		MuJoCoCameraInputs inputs;
		MuJoCoCameraOutputs outputs;
		State<MuJoCoCameraState> state;

		void pre_load() {}

		~MuJoCoCameraWorkload()
		{
			if (state->render_data)
			{
				mj_deleteData(state->render_data);
				state->render_data = nullptr;
				state->render_model = nullptr;
			}
		}

		void tick(const TickInfo&)
		{
			if (state->render_disabled)
			{
				// Avoid doing any work once we've opted out.
				outputs.png_data.set_size(0);
				return;
			}

			const mjModel* model = MuJoCoSceneRegistry::get().get_model(inputs.mujoco_scene_id);
			if (!model)
			{
				outputs.png_data.set_size(0);
				return;
			}

			if (state->render_model != model)
			{
				if (state->render_model != nullptr)
				{
					// Changing model implies a different mjData layout; we avoid realloc after init.
					ROBOTICK_WARNING("MuJoCoCameraWorkload: model changed after init; disabling render to avoid runtime allocation.");
					state->render_disabled = true;
					outputs.png_data.set_size(0);
					return;
				}

				// First-time allocation only; this buffer is reused on each tick.
				state->render_data = mj_makeData(model);
				state->render_model = model;
				state->render_context_ready = false;
			}

			if (!state->render_data)
			{
				outputs.png_data.set_size(0);
				return;
			}

			const mjModel* snapshot_model = nullptr;
			double snapshot_time = 0.0;
			// Copy the live sim state into our pre-allocated buffer.
			if (!MuJoCoSceneRegistry::get().copy_render_snapshot(inputs.mujoco_scene_id, state->render_data, snapshot_model, snapshot_time))
			{
				outputs.png_data.set_size(0);
				return;
			}

			if (!state->render_context_ready)
			{
				// Lazy-init the GL context once we have a valid model pointer.
				state->render_context_ready =
					state->render_context.init(snapshot_model, static_cast<int>(config.texture_width), static_cast<int>(config.texture_height));
				if (!state->render_context_ready)
				{
					outputs.png_data.set_size(0);
					return;
				}
			}

			if (!state->render_context.render_to_png(snapshot_model, state->render_data, config.camera_name.c_str(), outputs.png_data))
			{
				outputs.png_data.set_size(0);
			}
		}
	};
} // namespace robotick

#endif // ROBOTICK_PLATFORM_DESKTOP || ROBOTICK_PLATFORM_LINUX
