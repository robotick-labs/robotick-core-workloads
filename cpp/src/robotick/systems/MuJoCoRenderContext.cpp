// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#include "robotick/systems/MuJoCoRenderContext.h"

#if defined(ROBOTICK_PLATFORM_DESKTOP) || defined(ROBOTICK_PLATFORM_LINUX)

#include "robotick/api.h"
#include "robotick/framework/memory/StdApproved.h"
#include "robotick/systems/MuJoCoCallbacks.h"

#include <mujoco/mujoco.h>

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GL/gl.h>
#include <cstdlib>
#include <cstring>

namespace robotick
{
	namespace
	{
		using PFNEGLGETPLATFORMDISPLAYEXTPROC = EGLDisplay (*)(EGLenum, void*, const EGLint*);
		using PFNEGLQUERYDEVICESEXTPROC = EGLBoolean (*)(EGLint, EGLDeviceEXT*, EGLint*);

		PFNEGLGETPLATFORMDISPLAYEXTPROC get_platform_display_ext()
		{
			return reinterpret_cast<PFNEGLGETPLATFORMDISPLAYEXTPROC>(eglGetProcAddress("eglGetPlatformDisplayEXT"));
		}

		PFNEGLQUERYDEVICESEXTPROC query_devices_ext()
		{
			return reinterpret_cast<PFNEGLQUERYDEVICESEXTPROC>(eglGetProcAddress("eglQueryDevicesEXT"));
		}

		EGLDisplay create_surfaceless_display()
		{
			auto get_platform = get_platform_display_ext();
			if (!get_platform)
				return EGL_NO_DISPLAY;

			// Prefer device-backed EGL if available.
			auto query_devices = query_devices_ext();
			if (query_devices)
			{
				EGLDeviceEXT devices[16];
				EGLint device_count = 0;
				if (query_devices(16, devices, &device_count) == EGL_TRUE && device_count > 0)
				{
					const char* device_env = ::getenv("MUJOCO_EGL_DEVICE_ID");
					int device_index = 0;
					if (device_env)
						device_index = ::atoi(device_env);
					if (device_index < 0)
						device_index = 0;
					if (device_index >= device_count)
						device_index = 0;

					EGLDisplay display = get_platform(EGL_PLATFORM_DEVICE_EXT, devices[device_index], nullptr);
					if (display != EGL_NO_DISPLAY)
						return display;
				}
			}

			// Fallback: surfaceless Mesa platform.
			return get_platform(EGL_PLATFORM_SURFACELESS_MESA, EGL_DEFAULT_DISPLAY, nullptr);
		}

	} // namespace

	MuJoCoRenderContext::~MuJoCoRenderContext()
	{
		shutdown();
	}

	bool MuJoCoRenderContext::init(const mjModel* model, int width, int height)
	{
		if (!model || width <= 0 || height <= 0)
			return false;

		mujoco_callbacks::install();

		if (initialized_ && model_ == model && width_ == width && height_ == height)
			return true;

		shutdown();

		width_ = width;
		height_ = height;
		model_ = model;

		mjModel* mutable_model = const_cast<mjModel*>(model_);
		if (mutable_model)
		{
			if (mutable_model->vis.global.offwidth < width_)
				mutable_model->vis.global.offwidth = width_;
			if (mutable_model->vis.global.offheight < height_)
				mutable_model->vis.global.offheight = height_;
			mutable_model->vis.quality.offsamples = 0;
		}

		if (!init_gl_context())
			return false;

		scene_ = new mjvScene();
		option_ = new mjvOption();
		camera_ = new mjvCamera();
		context_ = new mjrContext();
		viewport_ = new mjrRect();

		mjv_defaultScene(scene_);
		mjv_makeScene(model_, scene_, 1000);
		mjv_defaultOption(option_);
		mjv_defaultCamera(camera_);
		mjr_defaultContext(context_);
		mjr_makeContext(model_, context_, mjFONTSCALE_100);
		mjr_resizeOffscreen(width_, height_, context_);

		update_viewport(width_, height_);
		mjr_setBuffer(mjFB_OFFSCREEN, context_);

		scene_ready_ = true;
		context_ready_ = true;
		initialized_ = true;
		return true;
	}

	void MuJoCoRenderContext::shutdown()
	{
		if (context_)
		{
			mjr_freeContext(context_);
			delete context_;
			context_ = nullptr;
		}

		if (scene_)
		{
			mjv_freeScene(scene_);
			delete scene_;
			scene_ = nullptr;
		}

		delete option_;
		delete camera_;
		delete viewport_;
		option_ = nullptr;
		camera_ = nullptr;
		viewport_ = nullptr;

		scene_ready_ = false;
		context_ready_ = false;

		destroy_gl_context();

		model_ = nullptr;
		width_ = 0;
		height_ = 0;
		initialized_ = false;
	}

	bool MuJoCoRenderContext::render_to_rgb(const mjModel* model,
		const mjData* data,
		const char* camera_name,
		uint8_t* out_rgb,
		size_t out_capacity,
		size_t& out_size,
		int& out_width,
		int& out_height,
		bool use_window_buffer)
	{
		out_size = 0;
		out_width = 0;
		out_height = 0;
		if (!model || !data || !out_rgb || out_capacity == 0)
		{
			ROBOTICK_WARNING("MuJoCoRenderContext: render_to_rgb called with null model/data.");
			return false;
		}

		if (!init(model, width_ > 0 ? width_ : 640, height_ > 0 ? height_ : 480))
		{
			ROBOTICK_WARNING("MuJoCoRenderContext: init failed in render_to_rgb.");
			return false;
		}

		if (!eglMakeCurrent(egl_display_, egl_surface_, egl_surface_, egl_context_))
		{
			ROBOTICK_WARNING("MuJoCoRenderContext: eglMakeCurrent failed in render_to_rgb.");
			return false;
		}

		mjr_setBuffer(mjFB_OFFSCREEN, context_);
		const mjrRect max_viewport = mjr_maxViewport(context_);
		if (max_viewport.width <= 0 || max_viewport.height <= 0)
		{
			ROBOTICK_WARNING("MuJoCoRenderContext: offscreen buffer unavailable (max viewport %dx%d).", max_viewport.width, max_viewport.height);
			return false;
		}

		int render_width = width_;
		int render_height = height_;
		if (render_width > max_viewport.width || render_height > max_viewport.height)
		{
			ROBOTICK_WARNING("MuJoCoRenderContext: clamping render size %dx%d to max %dx%d.",
				render_width,
				render_height,
				max_viewport.width,
				max_viewport.height);
			render_width = (render_width < max_viewport.width) ? render_width : max_viewport.width;
			render_height = (render_height < max_viewport.height) ? render_height : max_viewport.height;
		}

		update_viewport(render_width, render_height);

		if (viewport_->width <= 0 || viewport_->height <= 0)
		{
			ROBOTICK_WARNING("MuJoCoRenderContext: invalid viewport size %dx%d.", viewport_->width, viewport_->height);
			return false;
		}

		glViewport(0, 0, viewport_->width, viewport_->height);

		if (camera_name && camera_name[0] != '\0')
		{
			const int cam_id = mj_name2id(model, mjOBJ_CAMERA, camera_name);
			if (cam_id >= 0)
			{
				camera_->type = mjCAMERA_FIXED;
				camera_->fixedcamid = cam_id;
			}
			else
			{
				ROBOTICK_WARNING("MuJoCoRenderContext: camera '%s' not found; using default view", camera_name);
				mjv_defaultCamera(camera_);
			}
		}
		else
		{
			mjv_defaultCamera(camera_);
		}

		mjv_updateScene(model, const_cast<mjData*>(data), option_, nullptr, camera_, mjCAT_ALL, scene_);
		mjr_render(*viewport_, scene_, context_);

		// Re-bind the buffer before readback to avoid stale state.
		(void)use_window_buffer;
		mjr_setBuffer(mjFB_OFFSCREEN, context_);

		const size_t byte_count = static_cast<size_t>(viewport_->width * viewport_->height * 3);
		if (out_capacity < byte_count)
		{
			ROBOTICK_WARNING("MuJoCoRenderContext: output RGB buffer capacity %zu is smaller than required %zu.", out_capacity, byte_count);
			return false;
		}

		mjr_readPixels(out_rgb, nullptr, *viewport_, context_);
		out_width = viewport_->width;
		out_height = viewport_->height;
		out_size = byte_count;

		return true;
	}

	bool MuJoCoRenderContext::debug_clear_and_read_blue(
		uint8_t* out_rgb, size_t out_capacity, size_t& out_size, int& out_width, int& out_height, bool /*use_window_buffer*/)
	{
		out_size = 0;
		out_width = 0;
		out_height = 0;

		if (!out_rgb || out_capacity == 0 || !eglMakeCurrent(egl_display_, egl_surface_, egl_surface_, egl_context_))
			return false;

		mjr_setBuffer(mjFB_OFFSCREEN, context_);
		const mjrRect max_viewport = mjr_maxViewport(context_);
		if (max_viewport.width <= 0 || max_viewport.height <= 0)
		{
			ROBOTICK_WARNING("MuJoCoRenderContext: max viewport unavailable for debug clear.");
			return false;
		}

		int target_w = width_ > 0 ? width_ : 640;
		int target_h = height_ > 0 ? height_ : 480;
		if (target_w > max_viewport.width)
			target_w = max_viewport.width;
		if (target_h > max_viewport.height)
			target_h = max_viewport.height;

		update_viewport(target_w, target_h);

		glViewport(0, 0, viewport_->width, viewport_->height);
		glClearColor(0.0f, 0.0f, 1.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		const size_t byte_count = static_cast<size_t>(viewport_->width * viewport_->height * 3);
		if (out_capacity < byte_count)
		{
			ROBOTICK_WARNING("MuJoCoRenderContext: output RGB buffer capacity %zu is smaller than required %zu.", out_capacity, byte_count);
			return false;
		}
		glFinish();
		mjr_readPixels(out_rgb, nullptr, *viewport_, context_);
		out_width = viewport_->width;
		out_height = viewport_->height;
		out_size = byte_count;
		return true;
	}

	bool MuJoCoRenderContext::init_gl_context()
	{
		EGLDisplay display = create_surfaceless_display();
		if (display == EGL_NO_DISPLAY)
			return false;

		if (!eglInitialize(display, nullptr, nullptr))
			return false;

		auto try_create = [&](EGLint renderable_type, EGLenum api, const EGLint* ctx_attribs) -> bool
		{
			EGLint cfg_attribs[] = {EGL_SURFACE_TYPE,
				EGL_PBUFFER_BIT,
				EGL_RENDERABLE_TYPE,
				renderable_type,
				EGL_RED_SIZE,
				8,
				EGL_GREEN_SIZE,
				8,
				EGL_BLUE_SIZE,
				8,
				EGL_ALPHA_SIZE,
				8,
				EGL_DEPTH_SIZE,
				24,
				EGL_STENCIL_SIZE,
				8,
				EGL_NONE};

			EGLConfig config = nullptr;
			EGLint num_configs = 0;
			if (!eglChooseConfig(display, cfg_attribs, &config, 1, &num_configs) || num_configs < 1)
				return false;

			if (!eglBindAPI(api))
				return false;

			EGLContext context = eglCreateContext(display, config, EGL_NO_CONTEXT, ctx_attribs);
			if (context == EGL_NO_CONTEXT)
				return false;

			if (!eglMakeCurrent(display, EGL_NO_SURFACE, EGL_NO_SURFACE, context))
			{
				eglDestroyContext(display, context);
				return false;
			}

			egl_display_ = display;
			egl_surface_ = EGL_NO_SURFACE;
			egl_context_ = context;
			return true;
		};

		// Prefer desktop OpenGL; fall back to GLES2 if unavailable.
		const EGLint gl_ctx_attribs[] = {EGL_NONE};
		if (try_create(EGL_OPENGL_BIT, EGL_OPENGL_API, gl_ctx_attribs))
		{
			glViewport(0, 0, width_, height_);
			return true;
		}

		const EGLint gles_ctx_attribs[] = {EGL_CONTEXT_CLIENT_VERSION, 2, EGL_NONE};
		if (try_create(EGL_OPENGL_ES2_BIT, EGL_OPENGL_ES_API, gles_ctx_attribs))
		{
			glViewport(0, 0, width_, height_);
			return true;
		}

		eglTerminate(display);
		egl_display_ = EGL_NO_DISPLAY;
		return false;
	}

	void MuJoCoRenderContext::destroy_gl_context()
	{
		if (egl_display_ != EGL_NO_DISPLAY)
		{
			eglMakeCurrent(egl_display_, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
			if (egl_context_ != EGL_NO_CONTEXT)
			{
				eglDestroyContext(egl_display_, egl_context_);
				egl_context_ = EGL_NO_CONTEXT;
			}
			if (egl_surface_ != EGL_NO_SURFACE)
			{
				eglDestroySurface(egl_display_, egl_surface_);
				egl_surface_ = EGL_NO_SURFACE;
			}
			eglTerminate(egl_display_);
			egl_display_ = EGL_NO_DISPLAY;
		}
	}

	void MuJoCoRenderContext::update_viewport(int width, int height)
	{
		if (!viewport_)
			return;
		viewport_->left = 0;
		viewport_->bottom = 0;
		viewport_->width = width;
		viewport_->height = height;
	}

	void MuJoCoRenderContext::ensure_scene_initialized(const mjModel* model)
	{
		if (scene_ready_ && model_ == model)
			return;

		if (scene_)
		{
			mjv_freeScene(scene_);
			delete scene_;
			scene_ = nullptr;
		}

		scene_ = new mjvScene();
		mjv_defaultScene(scene_);
		mjv_makeScene(model, scene_, 1000);
		scene_ready_ = true;
	}
} // namespace robotick

#endif
