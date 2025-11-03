// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "robotick/api.h"
#include "robotick/systems/audio/AudioFrame.h"
#include "robotick/systems/audio/AudioSystem.h"
#include "robotick/systems/auditory/CochlearFrame.h"
#include "robotick/systems/auditory/SourceCandidate.h"

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
		bool draw_source_candidates = true;
	};

	struct CochlearVisualizerInputs
	{
		CochlearFrame cochlear_frame;
		SourceCandidates8 source_candidates;
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

		// simple colour palette
		static inline SDL_Color palette(uint8_t idx)
		{
			static const SDL_Color colors[] = {
				{255, 64, 64, 255},	 // red
				{64, 255, 64, 255},	 // green
				{64, 128, 255, 255}, // blue
				{255, 200, 64, 255}, // yellow
				{255, 96, 255, 255}, // magenta
				{64, 255, 255, 255}, // cyan
				{255, 160, 64, 255}, // orange
				{255, 255, 255, 255} // white
			};
			return colors[idx % 8];
		}

		inline float freq_to_y(float hz, float fmin, float fmax, int height) const
		{
			if (hz <= 0.0f)
				hz = fmin;
			hz = std::clamp(hz, fmin, fmax);
			float norm;
			if (config.log_scale)
				norm = (std::log(hz) - std::log(fmin)) / (std::log(fmax) - std::log(fmin));
			else
				norm = (hz - fmin) / (fmax - fmin);
			return (1.0f - norm) * (float)(height - 1);
		}

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
			std::memmove(s.pixels.data(), s.pixels.data() + 4, (size_t)(w - 1) * h * 4);

			// === Write new column on far-right edge ===

			for (int y = 0; y < draw_bands; ++y)
			{
				float v = inputs.cochlear_frame.envelope[y] * config.visual_gain;

				if (config.log_scale)
				{
					v = std::log1p(v * 10.0f) / std::log1p(10.0f);
				}

				v = std::clamp(v, 0.0f, 1.0f);
				const Uint8 c = (Uint8)(v * 255.0f);

				const int tex_y = (h - 1 - y);
				const int idx = (tex_y * w + (w - 1)) * 4;
				s.pixels[idx + 0] = 255;
				s.pixels[idx + 1] = c;
				s.pixels[idx + 2] = c;
				s.pixels[idx + 3] = c;
			}

			// === Overlay source candidates directly into pixel column ===
			if (config.draw_source_candidates)
			{
				const float fmin = 50.0f;
				const float fmax = 3500.0f;

				for (size_t i = 0; i < inputs.source_candidates.size(); ++i)
				{
					const auto& sc = inputs.source_candidates[i];
					if (sc.pitch_hz <= 0.0f)
						continue;

					const SDL_Color col = palette(sc.id);
					const float half_bw = 0.5f * sc.bandwidth_hz;

					const float f_lo = std::max(fmin, sc.pitch_hz - half_bw);
					const float f_hi = std::min(fmax, sc.pitch_hz + half_bw);

					const int y0 = (int)std::round(freq_to_y(sc.pitch_hz, fmin, fmax, h));
					const int ylo = (int)std::round(freq_to_y(f_lo, fmin, fmax, h));
					const int yhi = (int)std::round(freq_to_y(f_hi, fmin, fmax, h));

					// Write into pixel buffer (far-right column)
					auto paint_pixel = [&](int yy, int thickness)
					{
						for (int t = -thickness; t <= thickness; ++t)
						{
							const int tex_y = std::clamp(h - 1 - (yy + t), 0, h - 1);
							const int idx = (tex_y * w + (w - 1)) * 4;
							s.pixels[idx + 0] = col.r;
							s.pixels[idx + 1] = col.g;
							s.pixels[idx + 2] = col.b;
							s.pixels[idx + 3] = 255;
						}
					};

					paint_pixel(y0, 1);	 // bold F0
					paint_pixel(ylo, 0); // lower bound
					paint_pixel(yhi, 0); // upper bound
				}
			}

			// Update texture contents
			SDL_UpdateTexture(s.texture, nullptr, s.pixels.data(), w * 4);

			// === Stretch cochlear texture to viewport ===
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
