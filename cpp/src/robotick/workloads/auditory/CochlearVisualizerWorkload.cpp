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
#include <vector>

namespace robotick
{
	struct CochlearVisualizerConfig
	{
		float window_seconds = 5.0f; // horizontal duration to retain (seconds)
		int viewport_width = 800;	 // window width (pixels)
		int viewport_height = 400;	 // window height (pixels)
		bool log_scale = true;		 // log compression for visibility
		float visual_gain = 10.0f;	 // extra gain to brighten low values
		float fade_keep = 0.98f;	 // per-frame decay for older columns
	};

	struct CochlearVisualizerInputs
	{
		CochlearFrame cochlear_frame;
	};

	struct CochlearVisualizerState
	{
		SDL_Window* window = nullptr;
		SDL_Renderer* renderer = nullptr;
		SDL_Texture* texture = nullptr;

		int tex_w = 0;	// texture columns (time)
		int tex_h = 0;	// texture rows (bands)
		int head_x = 0; // circular write position

		// Approx frames/sec from cochlea tick rate; can be updated externally if needed.
		float frame_rate = 86.13f;

		std::vector<uint8_t> pixels; // RGBA interleaved

		void init_window(const CochlearVisualizerConfig& cfg, int num_bands)
		{
			if (SDL_Init(SDL_INIT_VIDEO) < 0)
			{
				fprintf(stderr, "SDL_Init failed: %s\n", SDL_GetError());
				return;
			}

			window = SDL_CreateWindow(
				"Cochlear Visualizer", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, cfg.viewport_width, cfg.viewport_height, SDL_WINDOW_SHOWN);

			if (!window)
			{
				fprintf(stderr, "SDL_CreateWindow failed: %s\n", SDL_GetError());
				return;
			}

			renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
			if (!renderer)
			{
				fprintf(stderr, "SDL_CreateRenderer failed: %s\n", SDL_GetError());
				return;
			}

			// Compute the texture dimensions: rows = bands, cols = seconds * frame_rate.
			const int cols_needed = std::max(1, (int)std::lround(frame_rate * cfg.window_seconds));
			tex_w = std::max(cfg.viewport_width, cols_needed); // ensure plenty of scrollback even if window is narrow
			tex_h = num_bands;

			texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_STREAMING, tex_w, tex_h);

			if (!texture)
			{
				fprintf(stderr, "SDL_CreateTexture failed: %s\n", SDL_GetError());
				return;
			}

			// Ensure no unexpected alpha blending darkens the image.
			SDL_SetTextureBlendMode(texture, SDL_BLENDMODE_NONE);

			// Map logical rendering coordinates to the texture size, then scale to window.
			SDL_RenderSetLogicalSize(renderer, tex_w, tex_h);

			pixels.assign((size_t)tex_w * (size_t)tex_h * 4, 0);

			// Optional: prime with a faint gray so you can see something immediately.
			for (size_t i = 0; i < pixels.size(); i += 4)
			{
				pixels[i + 0] = 255;
				pixels[i + 1] = 32;
				pixels[i + 2] = 32;
				pixels[i + 3] = 32;
			}
			SDL_UpdateTexture(texture, nullptr, pixels.data(), tex_w * 4);
			SDL_RenderClear(renderer);
			SDL_RenderCopy(renderer, texture, nullptr, nullptr);
			SDL_RenderPresent(renderer);
		}

		void shutdown()
		{
			if (texture)
			{
				SDL_DestroyTexture(texture);
				texture = nullptr;
			}
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
	};

	struct CochlearVisualizerWorkload
	{
		CochlearVisualizerConfig config;
		CochlearVisualizerInputs inputs;
		State<CochlearVisualizerState> state;

		void load()
		{
			// Use capacity for vertical texture allocation so we never underrun height.
			const int num_bands = (int)inputs.cochlear_frame.envelope.capacity();
			state->init_window(config, num_bands);
		}

		void tick(const TickInfo&)
		{
			auto& s = state.get();
			if (!s.renderer || !s.texture)
				return;

			const int bands_size = (int)inputs.cochlear_frame.envelope.size();
			if (bands_size <= 0)
				return;

			// Advance write column (circular)
			s.head_x = (s.head_x + 1) % std::max(1, s.tex_w);

			// Map most-recent envelope vector into grayscale column at head_x.
			const int draw_bands = std::min(bands_size, s.tex_h);
			for (int y = 0; y < draw_bands; ++y)
			{
				float v = inputs.cochlear_frame.envelope[y] * config.visual_gain;
				if (config.log_scale)
					v = std::log1p(v * 10.0f) / std::log1p(10.0f);
				v = std::clamp(v, 0.0f, 1.0f);
				const Uint8 c = (Uint8)(v * 255.0f);

				const int tex_y = (s.tex_h - 1 - y); // low freq at bottom, high at top
				const int idx = (tex_y * s.tex_w + s.head_x) * 4;
				s.pixels[idx + 0] = 255;
				s.pixels[idx + 1] = c;
				s.pixels[idx + 2] = c;
				s.pixels[idx + 3] = c;
			}

			// Fade other columns slightly to create a trailing effect.
			const float keep = std::clamp(config.fade_keep, 0.0f, 1.0f);
			for (int x = 0; x < s.tex_w; ++x)
			{
				if (x == s.head_x)
					continue;
				for (int y = 0; y < s.tex_h; ++y)
				{
					const int idx = (y * s.tex_w + x) * 4;
					const Uint8 v0 = s.pixels[idx + 0];
					const Uint8 v1 = (Uint8)std::lround((float)v0 * keep);
					s.pixels[idx + 0] = 255;
					s.pixels[idx + 1] = v1;
					s.pixels[idx + 2] = v1;
					s.pixels[idx + 3] = v1;
				}
			}

			SDL_UpdateTexture(s.texture, nullptr, s.pixels.data(), s.tex_w * 4);
			SDL_RenderClear(s.renderer);
			SDL_RenderCopy(s.renderer, s.texture, nullptr, nullptr);
			SDL_RenderPresent(s.renderer);

			// Basic event pump so the window is responsive.
			SDL_Event e;
			while (SDL_PollEvent(&e))
			{
				if (e.type == SDL_QUIT)
				{
					state->shutdown(); // allow engine to continue, window closes
					break;
				}
			}
		}

		void stop() { state->shutdown(); }
	};
} // namespace robotick
