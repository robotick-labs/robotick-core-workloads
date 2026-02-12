// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#include "robotick/api.h"
#include "robotick/framework/containers/HeapVector.h"
#include "robotick/systems/Image.h"
#include "robotick/systems/Renderer.h"
#include "robotick/systems/audio/AudioFrame.h"
#include "robotick/systems/audio/AudioSystem.h"
#include "robotick/systems/auditory/CochlearFrame.h"
#include "robotick/systems/auditory/HarmonicPitch.h"
#include "robotick/systems/auditory/ProsodyFusion.h"

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
		bool draw_pitch_info_amplitude = true;
		bool draw_harmonics = true;
		float pitch_visual_gain = 1.0f;
		float pitch_min_amplitude = 0.2f;

		// If true: render offscreen and export PNG bytes to outputs.visualization_png
		// If false: present to the active display/window
		bool render_to_texture = true;
		float fusion_link_alpha_gain = 100.0f;
	};

	struct CochlearVisualizerInputs
	{
		CochlearFrame cochlear_frame;	// envelope[Nbands], band_center_hz[Nbands]
		HarmonicPitchResult pitch_info; // h1_f0_hz, harmonic_amplitudes[k]
		ProsodicSegmentBuffer speech_segments;
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

		static void draw_line_segment(Renderer& renderer, const Vec2& a, const Vec2& b, const float thickness, const Color& color)
		{
			const Vec2 diff = b - a;
			const float length = diff.length();
			if (length < 1e-3f)
			{
				const Vec2 min = {a.x - thickness * 0.5f, a.y - thickness * 0.5f};
				const Vec2 max = {a.x + thickness * 0.5f, a.y + thickness * 0.5f};
				renderer.draw_rect_filled(min, max, color);
				return;
			}

			const float nx = -diff.y / length;
			const float ny = diff.x / length;
			const Vec2 offset(nx * thickness * 0.5f, ny * thickness * 0.5f);
			const Vec2 p0 = a + offset;
			const Vec2 p1 = a - offset;
			const Vec2 p2 = b - offset;
			const Vec2 p3 = b + offset;

			renderer.draw_triangle_filled(p0, p1, p2, color);
			renderer.draw_triangle_filled(p0, p2, p3, color);
		}

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

					if (h > 1 && !config.draw_harmonics)
						continue;

					float a = 1.0f;
					if (config.draw_pitch_info_amplitude)
					{
						a = (amp - config.pitch_min_amplitude) * config.pitch_visual_gain;
						if (config.log_scale)
							a = log1pf(a * 10.0f) / log1pf(10.0f);
						a = clampf(a, 0.0f, 1.0f);
					}

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

			struct SegmentOverlay
			{
				const ProsodicSegment* segment = nullptr;
				bool draw_bars = false;
				Color curve_color = Colors::White;
				Color bar_color = Colors::White;
				float start_x = 0.0f;
				float end_x = 0.0f;
			};

			FixedVector<SegmentOverlay, 64> overlays;
			const float window_seconds = config.window_seconds;
			const float window_end = tick.time_now;
			const float window_start = window_end - window_seconds;
			const auto segment_has_span = [](const ProsodicSegment& segment)
			{
				return (segment.end_time_sec > segment.start_time_sec) && (segment.pitch_hz.size() > 0);
			};

			if (window_seconds > 0.0f)
			{
				auto add_overlay = [&](const ProsodicSegment& segment, bool draw_bars, const Color& curve_color, const Color& bar_color)
				{
					const float start_norm = (segment.start_time_sec - window_start) / window_seconds;
					const float end_norm = (segment.end_time_sec - window_start) / window_seconds;
					if (end_norm <= 0.0f || start_norm >= 1.0f || overlays.full())
					{
						return;
					}

					SegmentOverlay overlay;
					overlay.segment = &segment;
					overlay.draw_bars = draw_bars;
					overlay.curve_color = curve_color;
					overlay.bar_color = bar_color;
					overlay.start_x = clampf(start_norm, 0.0f, 1.0f) * static_cast<float>(config.viewport_width);
					overlay.end_x = clampf(end_norm, 0.0f, 1.0f) * static_cast<float>(config.viewport_width);
					overlays.add(overlay);
				};

				for (const ProsodicSegment& segment : inputs.speech_segments)
				{
					if (!segment_has_span(segment))
					{
						continue;
					}

					Color curve_color = Colors::Yellow;
					Color bar_color = Colors::Yellow;
					switch (segment.state)
					{
					case ProsodicSegmentState::Ongoing:
						curve_color = Colors::Yellow;
						bar_color = Colors::Yellow;
						break;
					case ProsodicSegmentState::Completed:
						curve_color = Colors::Orange;
						bar_color = Colors::Orange;
						break;
					case ProsodicSegmentState::Finalised:
						curve_color = Colors::Blue;
						bar_color = Colors::White;
						break;
					}

					const bool draw_bars = (segment.state == ProsodicSegmentState::Finalised) || !segment.words.empty();
					add_overlay(segment, draw_bars, curve_color, bar_color);
				}
			}

			// 4) Draw to renderer and either present (live) or capture PNG (offscreen)
			s.renderer.clear(Colors::Black);
			s.renderer.draw_image_rgba8888_fit(s.rgba.data(), s.tex_w, s.tex_h);

			const auto time_to_x = [&](float absolute_time_sec) -> float
			{
				if (window_seconds <= 0.0f)
				{
					return -1.0f;
				}
				const float norm = (absolute_time_sec - window_start) / window_seconds;
				if (norm < 0.0f || norm > 1.0f)
				{
					return -1.0f;
				}
				return clampf(norm, 0.0f, 1.0f) * static_cast<float>(config.viewport_width);
			};

			const auto freq_to_y = [&](float freq_hz) -> float
			{
				if (s.tex_h <= 1)
				{
					return -1.0f;
				}
				const float band_idx = hz_to_band_idx(inputs.cochlear_frame.band_center_hz, freq_hz);
				if (band_idx < 0.0f)
				{
					return -1.0f;
				}
				const float norm = clampf(band_idx / static_cast<float>(s.tex_h - 1), 0.0f, 1.0f);
				const float viewport_height = static_cast<float>(config.viewport_height);
				return (1.0f - norm) * viewport_height;
			};

			const float curve_thickness = 3.0f;
			for (const SegmentOverlay& overlay : overlays)
			{
				const ProsodicSegment& segment = *overlay.segment;
				const size_t sample_count = segment.pitch_hz.size();
				if (sample_count == 0)
				{
					continue;
				}

				const float segment_duration = segment.end_time_sec - segment.start_time_sec;
				Vec2 prev_point = {};
				bool has_prev = false;

				const size_t mask_count = segment.pitch_link_mask.size();
				const size_t link_rms_count = segment.link_rms.size();

				for (size_t i = 0; i < sample_count; ++i)
				{
					const float freq_hz = segment.pitch_hz[i];
					if (freq_hz <= 0.0f)
					{
						has_prev = false;
						continue;
					}

					const float alpha = (sample_count <= 1) ? 0.0f : static_cast<float>(i) / static_cast<float>(sample_count - 1);
					const float sample_time = segment.start_time_sec + alpha * segment_duration;
					const float x = time_to_x(sample_time);
					const float y = freq_to_y(freq_hz);
					if (x < 0.0f || y < 0.0f)
					{
						has_prev = false;
						continue;
					}

					const Vec2 current_point = {x, y};
					if (has_prev)
					{
						const bool link_allowed = (i < mask_count) && (segment.pitch_link_mask[i] != 0);
						if (link_allowed)
						{
							const float link_rms = (i < link_rms_count) ? segment.link_rms[i] : segment.rms[i];
							const float alpha_scale = clampf(link_rms * config.fusion_link_alpha_gain, 0.05f, 1.0f);
							Color dynamic_color = overlay.curve_color;
							dynamic_color.a = static_cast<uint8_t>(alpha_scale * static_cast<float>(overlay.curve_color.a));
							draw_line_segment(s.renderer, prev_point, current_point, curve_thickness, dynamic_color);
						}
					}

					prev_point = current_point;
					has_prev = true;
				}
			}

			const float viewport_height = static_cast<float>(config.viewport_height);
			for (const SegmentOverlay& overlay : overlays)
			{
				if (!overlay.draw_bars)
				{
					continue;
				}

				const Vec2 start_bar_min = {overlay.start_x, 0.0f};
				const Vec2 start_bar_max = {overlay.start_x + 2.0f, viewport_height};
				s.renderer.draw_rect_filled(start_bar_min, start_bar_max, overlay.bar_color);

				const Vec2 end_bar_min = {overlay.end_x - 2.0f, 0.0f};
				const Vec2 end_bar_max = {overlay.end_x, viewport_height};
				s.renderer.draw_rect_filled(end_bar_min, end_bar_max, overlay.bar_color);

				if (overlay.segment && !overlay.segment->words.empty() && window_seconds > 0.0f)
				{
					for (size_t w = 0; w < overlay.segment->words.size(); ++w)
					{
						const TranscribedWord& word = overlay.segment->words[w];
						if (word.text.empty())
						{
							continue;
						}

						const float norm = (word.start_time_sec - window_start) / window_seconds;
						if (norm < 0.0f || norm > 1.0f)
						{
							continue;
						}

						const float word_x = clampf(norm, 0.0f, 1.0f) * static_cast<float>(config.viewport_width);
						const float line_offset = static_cast<float>((w % 2) * 12);
						const Vec2 label_pos = {word_x, 4.0f + line_offset};
						s.renderer.draw_text(word.text.c_str(), label_pos, 10.0f, TextAlign::Center, overlay.bar_color);
					}
				}
			}

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
