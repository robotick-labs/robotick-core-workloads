// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#pragma once

#if defined(ROBOTICK_PLATFORM_DESKTOP) || defined(ROBOTICK_PLATFORM_LINUX)

#include <cstddef>
#include <EGL/egl.h>

// Forward declarations from MuJoCo (keep in global namespace).
typedef struct mjModel_ mjModel;
typedef struct mjData_ mjData;
typedef struct mjvScene_ mjvScene;
typedef struct mjvOption_ mjvOption;
typedef struct mjvCamera_ mjvCamera;
typedef struct mjrContext_ mjrContext;
typedef struct mjrRect_ mjrRect;

namespace robotick
{
	class MuJoCoRenderContext
	{
	  public:
		MuJoCoRenderContext() = default;
		~MuJoCoRenderContext();

		bool init(const mjModel* model, int width, int height);
		void shutdown();
		bool is_ready() const { return initialized_; }

		bool render_to_rgb(
			const mjModel* model,
			const mjData* data,
			const char* camera_name,
			uint8_t* out_rgb,
			size_t out_capacity,
			size_t& out_size,
			int& out_width,
			int& out_height,
			bool use_window_buffer = false);

		// Test helper: clear the current framebuffer to solid blue and read back RGB.
		bool debug_clear_and_read_blue(
			uint8_t* out_rgb,
			size_t out_capacity,
			size_t& out_size,
			int& out_width,
			int& out_height,
			bool use_window_buffer = false);

	  private:
		bool init_gl_context();
		void destroy_gl_context();
		void update_viewport(int width, int height);
		void ensure_scene_initialized(const mjModel* model);

		bool initialized_ = false;
		bool owns_sdl_video_ = false;
		int width_ = 0;
		int height_ = 0;

		const mjModel* model_ = nullptr;

		EGLDisplay egl_display_ = EGL_NO_DISPLAY;
		EGLContext egl_context_ = EGL_NO_CONTEXT;
		EGLSurface egl_surface_ = EGL_NO_SURFACE;
		::mjvScene* scene_ = nullptr;
		::mjvOption* option_ = nullptr;
		::mjvCamera* camera_ = nullptr;
		::mjrContext* context_ = nullptr;
		::mjrRect* viewport_ = nullptr;

		bool scene_ready_ = false;
		bool context_ready_ = false;
	};
} // namespace robotick

#endif // ROBOTICK_PLATFORM_DESKTOP || ROBOTICK_PLATFORM_LINUX
