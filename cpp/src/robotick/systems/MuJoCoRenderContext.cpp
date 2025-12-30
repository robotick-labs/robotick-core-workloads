// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#include "robotick/systems/MuJoCoRenderContext.h"

#if defined(ROBOTICK_PLATFORM_DESKTOP) || defined(ROBOTICK_PLATFORM_LINUX)

#include "robotick/api.h"
#include "robotick/framework/memory/StdApproved.h"

#include <mujoco/mujoco.h>

#include <SDL2/SDL.h>
#include <cstring>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace robotick
{
	MuJoCoRenderContext::~MuJoCoRenderContext()
	{
		shutdown();
	}

	bool MuJoCoRenderContext::init(const mjModel* model, int width, int height)
	{
		if (!model || width <= 0 || height <= 0)
			return false;

		if (initialized_ && model_ == model && width_ == width && height_ == height)
			return true;

		shutdown();

		if (!init_sdl_video())
			return false;

		width_ = width;
		height_ = height;
		model_ = model;

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

	bool MuJoCoRenderContext::render_to_png(const mjModel* model, const mjData* data, const char* camera_name, ImagePng128k& out_png)
	{
		out_png.set_size(0);
		if (!model || !data)
			return false;

		if (!init(model, width_ > 0 ? width_ : 640, height_ > 0 ? height_ : 480))
			return false;

		if (SDL_GL_MakeCurrent(window_, gl_context_) != 0)
			return false;

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

		std_approved::vector<uint8_t> rgb;
		rgb.resize(static_cast<size_t>(width_ * height_ * 3));
		mjr_readPixels(rgb.data(), nullptr, *viewport_, context_);

		cv::Mat rgb_mat(height_, width_, CV_8UC3, rgb.data());
		cv::Mat rgb_flipped;
		cv::flip(rgb_mat, rgb_flipped, 0);

		cv::Mat bgr;
		cv::cvtColor(rgb_flipped, bgr, cv::COLOR_RGB2BGR);

		std_approved::vector<uint8_t> png_data;
		cv::imencode(".png", bgr, png_data);

		if (png_data.empty())
			return false;

		if (png_data.size() > out_png.capacity())
		{
			ROBOTICK_WARNING("MuJoCoRenderContext: PNG size %zu exceeds capacity %zu", png_data.size(), out_png.capacity());
			return false;
		}

		::memcpy(out_png.data(), png_data.data(), png_data.size());
		out_png.set_size(png_data.size());
		return true;
	}

	bool MuJoCoRenderContext::init_sdl_video()
	{
		if ((SDL_WasInit(SDL_INIT_VIDEO) & SDL_INIT_VIDEO) != 0)
			return true;

		if (SDL_InitSubSystem(SDL_INIT_VIDEO) < 0)
			return false;

		owns_sdl_video_ = true;
		return true;
	}

	bool MuJoCoRenderContext::init_gl_context()
	{
		SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
		SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);
		SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_COMPATIBILITY);

		window_ = SDL_CreateWindow(
			"MuJoCoRenderContext", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, width_, height_, SDL_WINDOW_OPENGL | SDL_WINDOW_HIDDEN);
		if (!window_)
			return false;

		gl_context_ = SDL_GL_CreateContext(window_);
		if (!gl_context_)
			return false;

		if (SDL_GL_MakeCurrent(window_, gl_context_) != 0)
			return false;

		return true;
	}

	void MuJoCoRenderContext::destroy_gl_context()
	{
		if (gl_context_)
		{
			SDL_GL_DeleteContext(gl_context_);
			gl_context_ = nullptr;
		}
		if (window_)
		{
			SDL_DestroyWindow(window_);
			window_ = nullptr;
		}
		if (owns_sdl_video_)
		{
			SDL_QuitSubSystem(SDL_INIT_VIDEO);
			owns_sdl_video_ = false;
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
