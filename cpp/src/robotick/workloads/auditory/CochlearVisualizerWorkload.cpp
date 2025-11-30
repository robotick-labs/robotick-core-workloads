// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#include "robotick/api.h"
#include "robotick/framework/containers/HeapVector.h"
#include "robotick/systems/Image.h"
#include "robotick/systems/Renderer.h"
#include "robotick/systems/audio/AudioFrame.h"
#include "robotick/systems/audio/AudioSystem.h"
#include "robotick/systems/auditory/CochlearFrame.h"
#include "robotick/systems/auditory/HarmonicPitch.h"

#include <cstring>

namespace robotick
{
	// ------------------------------------------------------------
	// Config / IO
	// ------------------------------------------------------------

	struct CochlearVisualizerConfig
	{
		float window_seconds = 5.0f; // visible history in seconds (x axis)
		int viewport_width = 512;	 // logical render width
		int viewport_height = 128;	 // logical render height
		bool log_scale = true;		 // log mapping of amplitudes
		float cochlear_visual_gain = 1.0f;

		bool draw_pitch_info = true;
		float pitch_visual_gain = 1.0f;
		float pitch_min_amplitude = 0.2f;

		// If true: render offscreen and export PNG bytes to outputs.visualization_png
		// If false: present to the active display/window
		bool render_to_texture = true;
	};

	struct CochlearVisualizerInputs
	{
		CochlearFrame cochlear_frame;	// envelope[Nbands], band_center_hz[Nbands]
		HarmonicPitchResult pitch_info; // h1_f0_hz, harmonic_amplitudes[k]
	};

	struct CochlearVisualizerOutputs
	{
		ImagePng128k visualization_png;
	};

	// ------------------------------------------------------------
	// Internal state (single allocation for the rolling image)
	// ------------------------------------------------------------

	struct CochlearVisualizerState
	{
		bool initialized = false;

		int tex_w = 0;			  // columns (history)
		int tex_h = 0;			  // rows (cochlear bands)
		HeapVector<uint8_t> rgba; // RGBA8888, size = tex_w * tex_h * 4 (desktop/test)

		Renderer renderer;
	};

	// ------------------------------------------------------------
	// Workload
	// ------------------------------------------------------------

	struct CochlearVisualizerWorkload
	{
		CochlearVisualizerConfig config;
		CochlearVisualizerInputs inputs;
		CochlearVisualizerOutputs outputs;
		State<CochlearVisualizerState> state;

		// --- helpers ---

		static inline float clampf(float v, float lo, float hi) { return (v < lo) ? lo : (v > hi) ? hi : v; }

		// Return fractional band index for frequency (for vertical placement)
		static float hz_to_band_idx(const AudioBuffer128& centers_hz, float hz)
		{
			const int n = static_cast<int>(centers_hz.size());
			if (n <= 1)
				return -1.0f;
			if (hz <= centers_hz[0] || hz >= centers_hz[n - 1])
				return -1.0f;

			for (int i = 0; i < n - 1; ++i)
			{
				const float f0 = centers_hz[i];
				const float f1 = centers_hz[i + 1];
				if (hz >= f0 && hz <= f1)
				{
					const float t = (hz - f0) / (f1 - f0);
					return static_cast<float>(i) + t;
				}
			}
			return -1.0f;
		}

		void initialize_renderer(float tick_rate_hz)
		{
			auto& s = state.get();
			if (s.initialized)
				return;

			const int bands = static_cast<int>(inputs.cochlear_frame.envelope.capacity());
			const int cols = robotick::max(1, static_cast<int>(lroundf(tick_rate_hz * config.window_seconds)));

			s.tex_w = cols;
			s.tex_h = bands;

			const size_t total_bytes = static_cast<size_t>(s.tex_w) * static_cast<size_t>(s.tex_h) * 4u;
			s.rgba.initialize(total_bytes);
			for (size_t i = 0; i < total_bytes; ++i)
			{
				s.rgba[i] = 0;
			}

			s.renderer.set_texture_only_size(static_cast<float>(config.viewport_width), static_cast<float>(config.viewport_height));
			s.renderer.set_viewport(static_cast<float>(config.viewport_width), static_cast<float>(config.viewport_height));
			s.renderer.init(config.render_to_texture);

			s.initialized = true;
		}

		void start(float tick_rate_hz) { initialize_renderer(tick_rate_hz); }

		void tick(const TickInfo& tick)
		{
			auto& s = state.get();
			if (!s.initialized)
			{
				initialize_renderer(tick.tick_rate_hz);
				if (!s.initialized)
					return;
			}

			if (s.tex_w <= 0 || s.tex_h <= 0)
				return;
			const int bands_size = static_cast<int>(inputs.cochlear_frame.envelope.size());
			if (bands_size <= 0)
				return;

			// 1) Scroll left by one column (preserve each row separately).
			if (s.tex_w > 1)
			{
				const size_t row_pitch = static_cast<size_t>(s.tex_w) * 4;
				const size_t shift_bytes = static_cast<size_t>(s.tex_w - 1) * 4;
				for (int row = 0; row < s.tex_h; ++row)
				{
					uint8_t* row_start = s.rgba.data() + static_cast<size_t>(row) * row_pitch;
					::memmove(row_start, row_start + 4, shift_bytes);
				}
			}

			// 2) Write new rightmost column from cochlear envelope (greyscale)
			const int draw_bands = robotick::min(bands_size, s.tex_h);
			for (int band = 0; band < draw_bands; ++band)
			{
				float a = inputs.cochlear_frame.envelope[band] * config.cochlear_visual_gain;
				if (config.log_scale)
				{
					a = log1pf(a * 10.0f) / log1pf(10.0f);
				}
				a = clampf(a, 0.0f, 1.0f);
				const uint8_t c = static_cast<uint8_t>(a * 255.0f);

				const int row = (s.tex_h - 1 - band);				 // low freq at bottom
				const int idx = (row * s.tex_w + (s.tex_w - 1)) * 4; // RGBA
				uint8_t* px = &s.rgba[static_cast<size_t>(idx)];
				px[0] = 255;
				px[1] = c;
				px[2] = c;
				px[3] = c;
			}

			// 3) Overlay harmonic markers on the new column (green/yellow)
			if (config.draw_pitch_info && inputs.pitch_info.h1_f0_hz > 0.0f)
			{
				const auto& p = inputs.pitch_info;
				for (size_t h = 1; h <= p.harmonic_amplitudes.size(); ++h)
				{
					float amp = p.harmonic_amplitudes[h - 1];
					if (amp <= 0.0f)
						continue;

					float a = (amp - config.pitch_min_amplitude) * config.pitch_visual_gain;
					if (config.log_scale)
						a = log1pf(a * 10.0f) / log1pf(10.0f);
					a = clampf(a, 0.0f, 1.0f);

					const uint8_t r = static_cast<uint8_t>(a * 64.0f);
					const uint8_t g = static_cast<uint8_t>(64.0f + a * (255.0f - 128.0f));

					const float f = p.h1_f0_hz * static_cast<float>(h);
					const float yf = hz_to_band_idx(inputs.cochlear_frame.band_center_hz, f);
					if (yf < 0.0f)
						continue;

					const int y = robotick::max(0, robotick::min(s.tex_h - 1, static_cast<int>(lroundf(yf))));
					const bool bold = (h == 1);
					const int thickness = bold ? 3 : 1;

					for (int t = 0; t < thickness; ++t)
					{
						const int row = robotick::max(0, robotick::min(s.tex_h - 1, (s.tex_h - 1 - (y + t))));
						const int idx = (row * s.tex_w + (s.tex_w - 1)) * 4;
						uint8_t* px = &s.rgba[static_cast<size_t>(idx)];
						px[0] = 255;
						px[1] = r;
						px[2] = g;
						px[3] = 0;
					}
				}
			}

			// 4) Draw to renderer and either present (live) or capture PNG (offscreen)
			s.renderer.clear(Colors::Black);
			s.renderer.draw_image_rgba8888_fit(s.rgba.data(), s.tex_w, s.tex_h);

			if (config.render_to_texture)
			{
				size_t png_size = 0;
				if (s.renderer.capture_as_png(outputs.visualization_png.data(), outputs.visualization_png.capacity(), png_size))
				{
					outputs.visualization_png.set_size(png_size);
				}
				else
				{
					ROBOTICK_WARNING("Failed to capture Cochlear visualizer PNG (capacity %zu bytes)", outputs.visualization_png.capacity());
					outputs.visualization_png.set_size(0);
				}
			}
			else
			{
				s.renderer.present();
			}
		}

		void stop() { state->renderer.cleanup(); }
	};

} // namespace robotick
