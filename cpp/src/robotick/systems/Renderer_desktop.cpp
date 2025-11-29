// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#if defined(ROBOTICK_PLATFORM_DESKTOP)

#include "robotick/api.h"
#include "robotick/framework/system/PlatformEvents.h"
#include "robotick/systems/Renderer.h"

#include <SDL2/SDL.h>
#include <SDL2/SDL2_gfxPrimitives.h>
#include <SDL2/SDL_ttf.h>

#include <opencv2/opencv.hpp>
#include <vector>

namespace robotick
{
	static SDL_Window* window = nullptr;
	static SDL_Renderer* renderer = nullptr;
	static SDL_Texture* blit_texture = nullptr; // cache for draw_image_rgba8888_fit
	static int blit_tex_w = 0;
	static int blit_tex_h = 0;

	static TTF_Font* font = nullptr;
	static int current_font_size = 0;

	bool is_windowed_mode()
	{
#if defined(_WIN32)
		return true; // Windows desktop
#elif defined(__linux__) && !defined(__arm__) && !defined(__aarch64__)
		return true;
#else
		return false; // Raspberry Pi, ESP32, other embedded
#endif
	}

	void Renderer::init(bool texture_only)
	{
		if (texture_only)
		{
			SDL_SetHint(SDL_HINT_RENDER_DRIVER, "software");
			SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "0"); // nearest

			if (SDL_Init(SDL_INIT_VIDEO) != 0)
				ROBOTICK_FATAL_EXIT("SDL_Init failed: %s", SDL_GetError());

			window =
				SDL_CreateWindow("OffscreenRenderer", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, physical_w, physical_h, SDL_WINDOW_HIDDEN);
			if (!window)
				ROBOTICK_FATAL_EXIT("SDL_CreateWindow (offscreen) failed: %s", SDL_GetError());

			renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_SOFTWARE);
			if (!renderer)
				ROBOTICK_FATAL_EXIT("SDL_CreateRenderer (offscreen) failed: %s", SDL_GetError());

			SDL_RenderSetLogicalSize(renderer, physical_w, physical_h);
			SDL_RenderSetIntegerScale(renderer, SDL_TRUE);

			int w = 0, h = 0;
			SDL_GetWindowSize(window, &w, &h);
			physical_w = w;
			physical_h = h;

			update_scale();
			return;
		}

		SDL_SetHint(SDL_HINT_RENDER_VSYNC, "1");
		SDL_SetHint(SDL_HINT_RENDER_DRIVER, "software");

		if (SDL_Init(SDL_INIT_VIDEO) != 0)
			ROBOTICK_FATAL_EXIT("SDL_Init failed: %s", SDL_GetError());

		if (TTF_Init() != 0)
			ROBOTICK_FATAL_EXIT("TTF_Init failed: %s", TTF_GetError());

		SDL_DisplayMode display_mode;
		if (SDL_GetCurrentDisplayMode(0, &display_mode) != 0)
			ROBOTICK_FATAL_EXIT("SDL_GetCurrentDisplayMode failed: %s", SDL_GetError());

		const bool is_windowed = is_windowed_mode();
		const uint32_t window_flags = is_windowed ? 0 : SDL_WINDOW_FULLSCREEN_DESKTOP;
		const int width = is_windowed ? display_mode.w / 4 : display_mode.w;
		const int height = is_windowed ? display_mode.h / 4 : display_mode.w;

		window = SDL_CreateWindow("Robotick Renderer", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, width, height, window_flags);
		if (!window)
			ROBOTICK_FATAL_EXIT("SDL_CreateWindow failed: %s", SDL_GetError());

		SDL_ShowWindow(window);
		SDL_RaiseWindow(window);

		SDL_GetWindowSize(window, &physical_w, &physical_h);

		renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_SOFTWARE);
		if (!renderer)
			ROBOTICK_FATAL_EXIT("SDL_CreateRenderer failed: %s", SDL_GetError());

		SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
		SDL_RenderClear(renderer);
		SDL_RenderPresent(renderer);

		update_scale();
	}

	void Renderer::cleanup()
	{
		if (font)
		{
			TTF_CloseFont(font);
			font = nullptr;
			current_font_size = 0;
		}

		if (blit_texture)
		{
			SDL_DestroyTexture(blit_texture);
			blit_texture = nullptr;
			blit_tex_w = 0;
			blit_tex_h = 0;
		}

		TTF_Quit();

		if (renderer)
		{
			SDL_DestroyRenderer(renderer);
			renderer = nullptr;
		}

		if (window)
		{
			SDL_DestroyWindow(window);
			window = nullptr;
		}

		SDL_Quit();
	}

	void Renderer::clear(const Color& color)
	{
		SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
		SDL_RenderClear(renderer);
	}

	void Renderer::present()
	{
		SDL_Window* win = SDL_GetWindowFromID(1); // or cache your SDL_Window*
		const Uint32 flags = win ? SDL_GetWindowFlags(win) : 0;

		const bool is_visible = win && (flags & SDL_WINDOW_SHOWN) && !(flags & SDL_WINDOW_MINIMIZED) && !(flags & SDL_WINDOW_HIDDEN);

		int w = 0, h = 0;
		if (window)
			SDL_GetWindowSize(window, &w, &h);

		if (w > 0 && h > 0 && is_visible)
		{
			SDL_RenderPresent(renderer);
		}

		poll_platform_events();
	}

	bool Renderer::capture_as_png(uint8_t* dst, size_t capacity, size_t& out_size)
	{
		out_size = 0;
		if (!dst || capacity == 0)
			return false;

		SDL_Surface* surface = SDL_CreateRGBSurfaceWithFormat(0, physical_w, physical_h, 32, SDL_PIXELFORMAT_ABGR8888);
		if (!surface)
			return false;

		SDL_RenderReadPixels(renderer, nullptr, SDL_PIXELFORMAT_ABGR8888, surface->pixels, surface->pitch);

		cv::Mat abgr(surface->h, surface->w, CV_8UC4, surface->pixels, surface->pitch);
		cv::Mat rgba;
		cv::cvtColor(abgr, rgba, cv::COLOR_BGRA2RGBA);

		// OpenCV only exposes STL vector-based encoders (no fixed buffer hook), so keep STL here and copy out afterward.
		std_approved::vector<uint8_t> png_data;
		cv::imencode(".png", rgba, png_data);

		SDL_FreeSurface(surface);

		if (png_data.empty())
			return false;

		if (png_data.size() > capacity)
		{
			ROBOTICK_WARNING("capture_as_png: PNG buffer (%zu bytes) exceeds destination capacity (%zu bytes)", png_data.size(), capacity);
			return false;
		}

		::memcpy(dst, png_data.data(), png_data.size());
		out_size = png_data.size();
		return true;
	}

	void Renderer::draw_ellipse_filled(const Vec2& center, const float rx, const float ry, const Color& color)
	{
		SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);

		const int cx_px = to_px_x(center.x);
		const int cy_px = to_px_y(center.y);
		const int rx_px = to_px_w(rx);
		const int ry_px = to_px_h(ry);

		filledEllipseRGBA(renderer, cx_px, cy_px, rx_px, ry_px, color.r, color.g, color.b, color.a);
	}

	void Renderer::draw_triangle_filled(const Vec2& p0, const Vec2& p1, const Vec2& p2, const Color& color)
	{
		const int x0 = to_px_x(p0.x);
		const int y0 = to_px_y(p0.y);
		const int x1 = to_px_x(p1.x);
		const int y1 = to_px_y(p1.y);
		const int x2 = to_px_x(p2.x);
		const int y2 = to_px_y(p2.y);

		filledTrigonRGBA(renderer, x0, y0, x1, y1, x2, y2, color.r, color.g, color.b, color.a);
	}

	void Renderer::draw_text(const char* text, const Vec2& pos, const float size, const TextAlign align, const Color& color)
	{
		if (!text || !*text || !renderer)
			return;

		const int font_size = static_cast<int>(size * scale);
		if (!font || current_font_size != font_size)
		{
			if (font)
			{
				TTF_CloseFont(font);
				font = nullptr;
			}

#if defined(__linux__)
			const char* font_path = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf";
#elif defined(_WIN32)
			const char* font_path = "C:\\Windows\\Fonts\\arial.ttf";
#else
			const char* font_path = "/System/Library/Fonts/Supplemental/Arial.ttf"; // macOS
#endif

			font = TTF_OpenFont(font_path, font_size);
			if (!font)
			{
				ROBOTICK_WARNING("Failed to load font at '%s': %s", font_path, TTF_GetError());
				return;
			}
			current_font_size = font_size;
		}

		SDL_Color sdl_color = {color.r, color.g, color.b, color.a};
		SDL_Surface* surface = TTF_RenderUTF8_Blended(font, text, sdl_color);
		if (!surface)
			return;

		SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer, surface);
		if (!texture)
		{
			SDL_FreeSurface(surface);
			return;
		}

		SDL_Rect dst;
		dst.w = surface->w;
		dst.h = surface->h;

		const int px = to_px_x(pos.x);
		const int py = to_px_y(pos.y);

		switch (align)
		{
		case TextAlign::TopLeft:
			dst.x = px;
			dst.y = py;
			break;
		case TextAlign::Center:
			dst.x = px - dst.w / 2;
			dst.y = py - dst.h / 2;
			break;
		default:
			dst.x = px;
			dst.y = py;
			break;
		}

		SDL_RenderCopy(renderer, texture, nullptr, &dst);
		SDL_DestroyTexture(texture);
		SDL_FreeSurface(surface);
	}

	// === New: raw RGBA blit, stretched to current viewport ===
	void Renderer::draw_image_rgba8888_fit(const uint8_t* pixels, int w, int h)
	{
		if (!pixels || w <= 0 || h <= 0 || !renderer)
			return;

		// (Re)create the cached texture if size changed
		if (!blit_texture || blit_tex_w != w || blit_tex_h != h)
		{
			if (blit_texture)
			{
				SDL_DestroyTexture(blit_texture);
				blit_texture = nullptr;
			}
			blit_texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_STREAMING, w, h);
			if (!blit_texture)
			{
				ROBOTICK_WARNING("draw_image_rgba8888_fit: failed to create texture: %s", SDL_GetError());
				return;
			}
			blit_tex_w = w;
			blit_tex_h = h;
		}

		// Upload pixels
		void* tex_pixels = nullptr;
		int pitch = 0;
		if (SDL_LockTexture(blit_texture, nullptr, &tex_pixels, &pitch) == 0)
		{
			// incoming is tightly-packed RGBA8888
			const uint8_t* src = pixels;
			uint8_t* dst = static_cast<uint8_t*>(tex_pixels);

			for (int y = 0; y < h; ++y)
			{
				::memcpy(dst + y * pitch, src + y * (w * 4), static_cast<size_t>(w * 4));
			}
			SDL_UnlockTexture(blit_texture);
		}
		else
		{
			ROBOTICK_WARNING("draw_image_rgba8888_fit: SDL_LockTexture failed: %s", SDL_GetError());
			return;
		}

		// Fit to the viewport region inside the window
		const SDL_Rect dst{
			offset_x,
			offset_y,
			static_cast<int>(logical_w * scale),
			static_cast<int>(logical_h * scale),
		};

		SDL_RenderCopy(renderer, blit_texture, nullptr, &dst);
	}
} // namespace robotick

#endif // ROBOTICK_PLATFORM_DESKTOP
