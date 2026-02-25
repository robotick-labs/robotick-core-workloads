// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#if defined(ROBOTICK_PLATFORM_DESKTOP) || defined(ROBOTICK_PLATFORM_LINUX)

#include "robotick/api.h"
#include "robotick/framework/concurrency/Sync.h"
#include "robotick/framework/containers/HeapVector.h"
#include "robotick/framework/memory/StdApproved.h"
#include "robotick/framework/strings/FixedString.h"
#include "robotick/systems/Image.h"
#include "robotick/systems/MuJoCoRenderContext.h"
#include "robotick/systems/MuJoCoSceneRegistry.h"

#include <mujoco/mujoco.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

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
		ImagePng256k png_data;
		uint32_t frame_count = 0;
	};

	struct MuJoCoCameraState
	{
		// Render context owns the GL/SDL state for this camera workload.
		MuJoCoRenderContext render_context;

		// Guards access to the render_data buffer and render_context usage.
		Mutex render_mutex;
		bool render_context_ready = false;

		// Once disabled, this workload produces empty output without reallocating.
		bool render_disabled = false;

		// Model pointer used to ensure the mjData buffer matches the scene layout.
		const mjModel* render_model = nullptr;

		// Pre-allocated mjData buffer used for thread-safe snapshot copies.
		mjData* render_data = nullptr;

		// Raw RGB output buffer and metadata (kept in state to avoid per-tick allocation).
		HeapVector<uint8_t> rgb_data;
		size_t rgb_size = 0;
		uint32_t rgb_width = 0;
		uint32_t rgb_height = 0;

		// Scratch buffer for PNG encoding; reserve once to avoid per-tick allocation.
		std_approved::vector<uint8_t> png_scratch;
	};

	struct MuJoCoCameraWorkload
	{
		MuJoCoCameraConfig config;
		MuJoCoCameraInputs inputs;
		MuJoCoCameraOutputs outputs;
		State<MuJoCoCameraState> state;

		static bool encode_png_from_rgb(
			const uint8_t* rgb, size_t rgb_size, int width, int height, ImagePng256k& out_png, std_approved::vector<uint8_t>& scratch)
		{
			out_png.set_size(0);
			if (!rgb || rgb_size == 0 || width <= 0 || height <= 0)
				return false;

			cv::Mat rgb_mat(height, width, CV_8UC3, const_cast<uint8_t*>(rgb));
			cv::Mat rgb_flipped;
			cv::flip(rgb_mat, rgb_flipped, 0);

			cv::Mat bgr;
			cv::cvtColor(rgb_flipped, bgr, cv::COLOR_RGB2BGR);

			// OpenCV only exposes STL vector-based encoders (no fixed buffer hook).
			scratch.clear();
			if (!cv::imencode(".png", bgr, scratch))
				return false;

			if (scratch.empty() || scratch.size() > out_png.capacity())
				return false;

			out_png.set(scratch.data(), scratch.size());
			return true;
		}

		void tick(const TickInfo&)
		{

			ROBOTICK_ASSERT(config.texture_width > 0);
			ROBOTICK_ASSERT(config.texture_height > 0);

			if (state->render_disabled)
			{
				// Avoid doing any work once we've opted out.
				ROBOTICK_WARNING_ONCE("MuJoCoCameraWorkload: rendering disabled; skipping tick.");
				return;
			}

			const mjModel* model = MuJoCoSceneRegistry::get().get_model(inputs.mujoco_scene_id);
			if (!model)
			{
				ROBOTICK_WARNING("MuJoCoCameraWorkload: null model for scene ID %u", inputs.mujoco_scene_id);
				return;
			}

			if (state->render_model != model)
			{
				if (state->render_model != nullptr)
				{
					// Changing model implies a different mjData layout; we avoid realloc after init.
					ROBOTICK_WARNING("MuJoCoCameraWorkload: model changed after init; disabling render to avoid runtime allocation.");
					state->render_disabled = true;
					return;
				}

				// First-time allocation only; this buffer is reused on each tick.
				state->render_data = mj_makeData(model);
				if (!state->render_data)
				{
					ROBOTICK_WARNING("MuJoCoCameraWorkload: failed to allocate mjData; disabling render.");
					state->render_disabled = true;
					return;
				}
				state->render_model = model;
				state->render_context_ready = false;
			}

			if (!state->render_data)
			{
				ROBOTICK_WARNING("MuJoCoCameraWorkload: null render_data");
				return;
			}

			const mjModel* snapshot_model = nullptr;
			double snapshot_time = 0.0;
			// Copy the live sim state into our pre-allocated buffer.
			if (!MuJoCoSceneRegistry::get().copy_render_snapshot(inputs.mujoco_scene_id, state->render_data, snapshot_model, snapshot_time))
			{
				ROBOTICK_WARNING("MuJoCoCameraWorkload: copy_render_snapshot failed; disabling render.");
				state->render_disabled = true;
				return;
			}
			if (!snapshot_model || snapshot_model != state->render_model)
			{
				ROBOTICK_WARNING("MuJoCoCameraWorkload: snapshot model mismatch; disabling render to avoid corruption.");
				state->render_disabled = true;
				return;
			}

			// Render the camera view into our RGB buffer
			{
				LockGuard render_lock(state->render_mutex);

				if (!state->render_context_ready)
				{
					// Lazy-init the GL context once we have a valid model pointer.
					state->render_context_ready =
						state->render_context.init(snapshot_model, static_cast<int>(config.texture_width), static_cast<int>(config.texture_height));
					if (!state->render_context_ready)
					{
						ROBOTICK_WARNING("MuJoCoCameraWorkload: render context init failed; disabling render.");
						state->render_disabled = true;
						return;
					}
				}

				if (state->png_scratch.capacity() == 0)
					state->png_scratch.reserve(ImagePng256k::capacity());

				const size_t rgb_capacity = static_cast<size_t>(config.texture_width * config.texture_height * 3);
				if (state->rgb_data.size() == 0)
				{
					state->rgb_data.initialize(rgb_capacity);
				}
				else if (state->rgb_data.size() != rgb_capacity)
				{
					ROBOTICK_WARNING("MuJoCoCameraWorkload: rgb_data size %zu does not match required %zu.", state->rgb_data.size(), rgb_capacity);
					state->render_disabled = true;
					return;
				}

				int rgb_width = 0;
				int rgb_height = 0;
				size_t rgb_size = 0;
				if (!state->render_context.render_to_rgb(snapshot_model,
						state->render_data,
						config.camera_name.c_str(),
						state->rgb_data.data(),
						state->rgb_data.size(),
						rgb_size,
						rgb_width,
						rgb_height,
						false))
				{
					state->rgb_size = 0;
					state->rgb_width = 0;
					state->rgb_height = 0;
					outputs.png_data.set_size(0);
					ROBOTICK_WARNING("MuJoCoCameraWorkload: render_to_rgb failed; output cleared.");
					return;
				}

				state->rgb_size = rgb_size;
				state->rgb_width = static_cast<uint32_t>(rgb_width);
				state->rgb_height = static_cast<uint32_t>(rgb_height);
			}

			if (!encode_png_from_rgb(state->rgb_data.data(),
					state->rgb_size,
					static_cast<int>(state->rgb_width),
					static_cast<int>(state->rgb_height),
					outputs.png_data,
					state->png_scratch))
			{
				outputs.png_data.set_size(0);
				ROBOTICK_WARNING("MuJoCoCameraWorkload: PNG encode failed; output cleared.");
				return;
			}

			outputs.frame_count++;
		}

		void stop()
		{
			state->render_context.shutdown();
			state->render_context_ready = false;
			state->render_disabled = false;
			if (state->render_data)
			{
				mj_deleteData(state->render_data);
				state->render_data = nullptr;
				state->render_model = nullptr;
			}
		}
	};
} // namespace robotick

#endif // ROBOTICK_PLATFORM_DESKTOP || ROBOTICK_PLATFORM_LINUX
