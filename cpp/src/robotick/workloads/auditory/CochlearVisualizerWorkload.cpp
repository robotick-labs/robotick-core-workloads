// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "robotick/api.h"
#include "robotick/systems/audio/AudioFrame.h"
#include "robotick/systems/audio/AudioSystem.h"
#include "robotick/systems/auditory/CochlearFrame.h"

#include <SDL2/SDL.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

namespace robotick
{
	struct CochlearVisualizerConfig
	{
		float window_seconds = 5.0f; // horizontal duration shown (s)
		int viewport_width = 800;	 // window width (px)
		int viewport_height = 400;	 // window height (px)
		bool log_scale = true;
		float visual_gain = 1.0f;
	};

	struct CochlearVisualizerInputs
	{
		CochlearFrame cochlear_frame;
	};

	struct CochlearVisualizerState
	{
		bool has_initialized = false;

		SDL_Window* window = nullptr;
		SDL_Renderer* renderer = nullptr;
		SDL_Texture* texture = nullptr;

		int tex_w = 0;
		int tex_h = 0;

		std::vector<uint8_t> pixels; // RGBA

		void init_window(const CochlearVisualizerConfig& cfg, int num_bands, float tick_rate_hz)
		{
			has_initialized = true;

			if (SDL_Init(SDL_INIT_VIDEO) < 0)
			{
				ROBOTICK_FATAL_EXIT("SDL_Init failed: %s\n", SDL_GetError());
				return;
			}

			window = SDL_CreateWindow(
				"Cochlear Visualizer", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, cfg.viewport_width, cfg.viewport_height, SDL_WINDOW_SHOWN);

			renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
			if (!renderer)
			{
				ROBOTICK_FATAL_EXIT("SDL_CreateRenderer failed: %s\n", SDL_GetError());
				return;
			}

			// true texture width is purely determined by desired time window
			const int cols_needed = std::max(1, (int)std::lround(tick_rate_hz * cfg.window_seconds));
			tex_w = cols_needed;
			tex_h = num_bands;

			texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_STREAMING, tex_w, tex_h);
			SDL_SetTextureBlendMode(texture, SDL_BLENDMODE_NONE);

			pixels.assign((size_t)tex_w * (size_t)tex_h * 4, 0);
		}

		void shutdown()
		{
			if (texture)
				SDL_DestroyTexture(texture);
			if (renderer)
				SDL_DestroyRenderer(renderer);
			if (window)
				SDL_DestroyWindow(window);
			texture = nullptr;
			renderer = nullptr;
			window = nullptr;
			SDL_Quit();
		}
	};

	struct CochlearVisualizerWorkload
	{
		CochlearVisualizerConfig config;
		CochlearVisualizerInputs inputs;
		State<CochlearVisualizerState> state;

		void tick(const TickInfo& tick_info)
		{
			auto& s = state.get();

			if (!s.has_initialized)
			{
				const int num_bands = (int)inputs.cochlear_frame.envelope.capacity();
				s.init_window(config, num_bands, tick_info.tick_rate_hz);
			}

			if (!s.renderer || !s.texture)
				return;

			const int bands_size = (int)inputs.cochlear_frame.envelope.size();
			if (bands_size <= 0)
				return;

			const int draw_bands = std::min(bands_size, s.tex_h);
			const int w = s.tex_w;
			const int h = s.tex_h;

			// === Shift all pixels left by one column ===
			for (int y = 0; y < h; ++y)
			{
				uint8_t* row = &s.pixels[y * w * 4];
				std::memmove(row, row + 4, (w - 1) * 4);
			}

			// === Write new column on far-right edge ===
			for (int y = 0; y < draw_bands; ++y)
			{
				float v = inputs.cochlear_frame.envelope[y] * config.visual_gain;
				if (config.log_scale)
					v = std::log1p(v * 10.0f) / std::log1p(10.0f);
				v = std::clamp(v, 0.0f, 1.0f);
				const Uint8 c = (Uint8)(v * 255.0f);

				const int tex_y = (h - 1 - y);
				const int idx = (tex_y * w + (w - 1)) * 4;
				s.pixels[idx + 0] = 255;
				s.pixels[idx + 1] = c;
				s.pixels[idx + 2] = c;
				s.pixels[idx + 3] = c;
			}

			// Update texture contents
			SDL_UpdateTexture(s.texture, nullptr, s.pixels.data(), w * 4);

			// === Stretch the smaller texture to fill the viewport ===
			SDL_Rect dest{0, 0, config.viewport_width, config.viewport_height};
			SDL_RenderClear(s.renderer);
			SDL_RenderCopy(s.renderer, s.texture, nullptr, &dest);
			SDL_RenderPresent(s.renderer);

			// Handle events
			SDL_Event e;
			while (SDL_PollEvent(&e))
			{
				if (e.type == SDL_QUIT)
				{
					state->shutdown();
					break;
				}
			}
		}

		void stop() { state->shutdown(); }
	};
} // namespace robotick
