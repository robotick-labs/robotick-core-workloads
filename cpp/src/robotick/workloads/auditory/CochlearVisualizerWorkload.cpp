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
		float min_source_amplitude = 0.5f;
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

			texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING, tex_w, tex_h);
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
				{64, 255, 64, 255},	 // green
				{255, 255, 96, 255}, // magenta
				{64, 160, 255, 255}, // orange
				{255, 64, 64, 255},	 // blue
				{255, 255, 64, 255}, // cyan
				{64, 255, 200, 255}, // yellow
				{64, 64, 255, 255},	 // red
				{255, 64, 192, 255}	 // violet
			};
			return colors[idx % 8];
		}

		inline float hz_to_band_y(const AudioBuffer128& band_center_hz, float hz)
		{
			const int n = (int)band_center_hz.size();
			if (n <= 1)
				return -1.0f;

			// Clamp frequency to band range
			if (hz <= band_center_hz[0])
				return -1.0f;
			if (hz >= band_center_hz[n - 1])
				return -1.0f;

			// Find the nearest two bands
			for (int i = 0; i < n - 1; ++i)
			{
				const float f0 = band_center_hz[i];
				const float f1 = band_center_hz[i + 1];
				if (hz >= f0 && hz <= f1)
				{
					const float t = (hz - f0) / (f1 - f0);
					// Y = band index (bottom to top)
					return (i + t);
				}
			}

			return -1.0f;
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
				s.pixels[idx + 0] = c;
				s.pixels[idx + 1] = c;
				s.pixels[idx + 2] = c;
				s.pixels[idx + 3] = 255;
			}

			// === Overlay source candidates directly into pixel column ===
			if (config.draw_source_candidates)
			{
				for (size_t i = 0; i < inputs.source_candidates.size(); ++i)
				{
					const auto& sc = inputs.source_candidates[i];
					if (sc.pitch_hz <= 0.0f)
						continue;

					const SDL_Color col = palette(i);
					const float half_bw = 0.5f * sc.bandwidth_hz;

					const float f_lo = sc.pitch_hz - half_bw;
					const float f_hi = sc.pitch_hz + half_bw;

					const float y0f = hz_to_band_y(inputs.cochlear_frame.band_center_hz, sc.pitch_hz);
					if (y0f < 0.0f)
					{
						continue;
					}

					const float ylof = hz_to_band_y(inputs.cochlear_frame.band_center_hz, f_lo);
					const float yhif = hz_to_band_y(inputs.cochlear_frame.band_center_hz, f_hi);

					const int y0 = (int)std::round(y0f);
					const int ylo = (int)std::round(ylof);
					const int yhi = (int)std::round(yhif);

					// Write into pixel buffer (far-right column)
					auto paint_pixel = [&](int yy, const bool bold = false)
					{
						const int thickness = bold ? 2 : 1;

						for (int t = 0; t < thickness; ++t)
						{
							const int tex_y = std::clamp(h - 1 - (yy + t), 0, h - 1);
							const int idx = (tex_y * w + (w - 1)) * 4;

							s.pixels[idx + 0] = col.r;
							s.pixels[idx + 1] = col.g;
							s.pixels[idx + 2] = col.b;
							s.pixels[idx + 3] = 255; // keep opaque overall
						}
					};

					paint_pixel(y0, true); // bold F0
					paint_pixel(ylo);	   // lower bound
					paint_pixel(yhi);	   // upper bound
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
