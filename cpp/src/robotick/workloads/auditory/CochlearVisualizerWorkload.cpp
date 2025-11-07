// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#include "robotick/api.h"
#include "robotick/systems/audio/AudioFrame.h"
#include "robotick/systems/audio/AudioSystem.h"
#include "robotick/systems/auditory/CochlearFrame.h"
#include "robotick/systems/auditory/HarmonicPitch.h"

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
		float cochlear_visual_gain = 1.0f;
		bool draw_pitch_info = true;
		float pitch_visual_gain = 1.0f;
		float pitch_min_amplitude = 0.2f;
	};

	struct CochlearVisualizerInputs
	{
		CochlearFrame cochlear_frame;
		HarmonicPitchResult pitch_info;
	};

	struct CochlearVisualizerState
	{
		bool has_initialized = false;
		bool is_rendering_paused = false;

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

			window = SDL_CreateWindow("Cochlear Visualizer",
				SDL_WINDOWPOS_CENTERED,
				SDL_WINDOWPOS_CENTERED,
				cfg.viewport_width,
				cfg.viewport_height,
				SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);

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
			auto& state_ref = state.get();

			if (!state_ref.has_initialized)
			{
				const int num_bands = (int)inputs.cochlear_frame.envelope.capacity();
				state_ref.init_window(config, num_bands, tick_info.tick_rate_hz);
			}

			if (!state_ref.renderer || !state_ref.texture)
				return;

			const int bands_size = (int)inputs.cochlear_frame.envelope.size();
			if (bands_size <= 0)
				return;

			const int draw_bands = std::min(bands_size, state_ref.tex_h);
			const int tex_w = state_ref.tex_w;
			const int tex_h = state_ref.tex_h;

			// === Shift all pixels left by one column ===
			std::memmove(state_ref.pixels.data(), state_ref.pixels.data() + 4, (size_t)(tex_w - 1) * tex_h * 4);

			// === Write new column on far-right edge ===

			for (int band_id = 0; band_id < draw_bands; ++band_id)
			{
				float amplitude = inputs.cochlear_frame.envelope[band_id] * config.cochlear_visual_gain;

				if (config.log_scale)
				{
					amplitude = std::log1p(amplitude * 10.0f) / std::log1p(10.0f);
				}

				amplitude = clamp(amplitude, 0.0f, 1.0f);
				const Uint8 c = (Uint8)(amplitude * 255.0f);

				const int tex_y = (tex_h - 1 - band_id);
				const int idx = (tex_y * tex_w + (tex_w - 1)) * 4;
				state_ref.pixels[idx + 0] = c;
				state_ref.pixels[idx + 1] = c;
				state_ref.pixels[idx + 2] = c;
				state_ref.pixels[idx + 3] = 255;
			}

			// === Overlay source candidates directly into pixel column ===
			if (config.draw_pitch_info)
			{
				const HarmonicPitchResult& pitch_info = inputs.pitch_info;
				if (pitch_info.h1_f0_hz > 0.0f)
				{
					for (size_t harmonic_id = 1; harmonic_id <= pitch_info.harmonic_amplitudes.size(); harmonic_id++)
					{
						const float amplitude = pitch_info.harmonic_amplitudes[harmonic_id - 1];
						if (amplitude == 0.0f)
						{
							continue;
						}

						float amplitude_norm = (amplitude - config.pitch_min_amplitude) * config.pitch_visual_gain;
						if (config.log_scale)
						{
							amplitude_norm = std::log1p(amplitude_norm * 10.0f) / std::log1p(10.0f);
						}
						amplitude_norm = clamp(amplitude_norm, 0.0f, 1.0f);

						// -----------------------------------------------------------------------------
						// Colour: Lerp from dark-green {0,32,0} to lime {64,255,0} based on amplitude
						// -----------------------------------------------------------------------------
						const Uint8 red = static_cast<Uint8>(0.0f + amplitude_norm * 64.0f);
						const Uint8 green = static_cast<Uint8>(64.0f + amplitude_norm * (255.0f - 128.0f));
						const SDL_Color colour_for_source = {red, green, 0, 255};

						const float harmonic_frequency = pitch_info.h1_f0_hz * static_cast<float>(harmonic_id);
						const float draw_y_float = hz_to_band_y(inputs.cochlear_frame.band_center_hz, harmonic_frequency);
						if (draw_y_float < 0.0f)
						{
							continue;
						}

						const int draw_y = static_cast<int>(std::round(draw_y_float));

						const bool draw_bold = (harmonic_id == 1);
						const int thickness = draw_bold ? 3 : 1;

						for (int thickness_id = 0; thickness_id < thickness; ++thickness_id)
						{
							const int tex_y = clamp(tex_h - 1 - (draw_y + thickness_id), 0, tex_h - 1);
							const int idx = (tex_y * tex_w + (tex_w - 1)) * 4;

							state_ref.pixels[idx + 0] = colour_for_source.r;
							state_ref.pixels[idx + 1] = colour_for_source.g;
							state_ref.pixels[idx + 2] = colour_for_source.b;
							state_ref.pixels[idx + 3] = 255;
						}
					}
				}
			}

			// Update texture contents
			SDL_UpdateTexture(state_ref.texture, nullptr, state_ref.pixels.data(), tex_w * 4);

			// === Stretch cochlear texture to viewport ===
			int window_width, window_height;
			SDL_GetWindowSize(state_ref.window, &window_width, &window_height);

			SDL_Rect dest{0, 0, window_width, window_height};
			SDL_RenderClear(state_ref.renderer);
			SDL_RenderCopy(state_ref.renderer, state_ref.texture, nullptr, &dest);

			if (!state_ref.is_rendering_paused)
			{
				SDL_RenderPresent(state_ref.renderer);
			}

			// Handle events
			SDL_Event e;
			while (SDL_PollEvent(&e))
			{
				if (e.type == SDL_QUIT)
				{
					state->shutdown();
					break;
				}
				else if (e.type == SDL_KEYDOWN)
				{
					if (e.key.keysym.sym == SDLK_SPACE)
					{
						state_ref.is_rendering_paused = !state_ref.is_rendering_paused;
					}
				}
			}
		}

		void stop() { state->shutdown(); }
	};
} // namespace robotick
