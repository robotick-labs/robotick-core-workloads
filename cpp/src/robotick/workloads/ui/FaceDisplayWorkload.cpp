// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#include "robotick/api.h"
#include "robotick/systems/Image.h"
#include "robotick/systems/Renderer.h"

namespace robotick
{
	struct FaceDisplayConfig
	{
		float blink_min_interval_sec = 1.5f;
		float blink_max_interval_sec = 4.0f;

		// if true, produce PNG instead of rendering to window:
		bool render_to_texture = false;

		Vec2f look_offset_scale = {30.0f, -25.0f};
	};

	struct FaceDisplayInputs
	{
		Vec2f look_offset = {0.0f, 0.0f};
		bool blink_request = false;
		float max_eyes_open_norm = 1.0f; // 0.0 (closed) to 1.0 (fully wide open)
	};

	struct FaceDisplayOutputs
	{
		ImagePng16k face_png_data;
		// ^- size estimate, tune as needed
	};

	struct FaceDisplayState
	{
		bool prev_blink_request = false;
		float eye_blink_progress[2] = {0, 0};
		float next_blink_time[2] = {0, 0};

		bool has_init_renderer = false;
		Renderer renderer;
	};

	struct FaceDisplayWorkload
	{
		FaceDisplayConfig config;
		FaceDisplayInputs inputs;
		FaceDisplayOutputs outputs;
		State<FaceDisplayState> state;

		void setup() { schedule_blink_pair(0.0f); }

		void start(float)
		{
			auto& s = state.get();
			if (s.has_init_renderer)
				return;

			s.renderer.set_texture_only_size(800, 480);
			s.renderer.set_viewport(320, 240);
			s.renderer.init(config.render_to_texture);
			s.has_init_renderer = true;
		}

		void tick(const TickInfo& tick_info)
		{
			auto& s = state.get();

			// Update blink animations
			const float time_now_sec = tick_info.time_now;
			update_blinks(time_now_sec);

			// Draw face
			s.renderer.clear(Colors::White);
			draw_face(s.renderer);

			if (config.render_to_texture)
			{
				size_t png_size = 0;
				if (s.renderer.capture_as_png(outputs.face_png_data.data(), outputs.face_png_data.capacity(), png_size))
				{
					outputs.face_png_data.set_size(png_size);
				}
				else
				{
					outputs.face_png_data.set_size(0);
				}
			}
			else
			{
				s.renderer.present();
			}
		}

		void update_blinks(const float time_now_sec)
		{
			auto& s = state.get();
			auto& blink = s.eye_blink_progress;
			auto& next_time = s.next_blink_time;

			if (inputs.blink_request && !state->prev_blink_request)
			{
				next_time[0] = 0.0f; // bring the next blink forward to "now"
				next_time[1] = 0.0f;
			}

			state->prev_blink_request = inputs.blink_request;

			if (time_now_sec >= next_time[0] || time_now_sec >= next_time[1])
			{
				blink[0] = 1.0f;
				blink[1] = 1.0f;
				schedule_blink_pair(time_now_sec);
			}
			else
			{
				for (int i = 0; i < 2; ++i)
				{
					if (blink[i] > 0.0f)
					{
						blink[i] -= 0.15f;
						if (blink[i] < 0.0f)
							blink[i] = 0.0f;
					}
				}
			}
		}

		void schedule_blink_pair(const float time_now_sec)
		{
			auto& next_time = state->next_blink_time;
			const float min_sec = config.blink_min_interval_sec;
			const float max_sec = config.blink_max_interval_sec;
			const float random_interval = min_sec + ((float)rand() / RAND_MAX) * (max_sec - min_sec);
			const float max_eye_offset = 0.1f;

			next_time[0] = time_now_sec + random_interval + ((((float)rand() / RAND_MAX) * 2.0f - 1.0f) * max_eye_offset);
			next_time[1] = time_now_sec + random_interval + ((((float)rand() / RAND_MAX) * 2.0f - 1.0f) * max_eye_offset);
		}

		void draw_face(Renderer& r)
		{
			auto& blink = state->eye_blink_progress;
			const int center_y = 120;
			const int eye_w = 40;
			const int eye_h = 65;
			const int eye_spacing = 200;

			const Vec2 look_offset_screen(inputs.look_offset.x * config.look_offset_scale.x, inputs.look_offset.y * config.look_offset_scale.y);

			for (int i = 0; i < 2; ++i)
			{
				const int cx = 160 + (i == 0 ? -eye_spacing / 2 : eye_spacing / 2);
				const float scale_y = (1.0f - 0.8f * blink[i]) * clamp(inputs.max_eyes_open_norm, 0.0f, 1.0f);
				draw_eye(r, cx + look_offset_screen.x, center_y + look_offset_screen.y, eye_w, static_cast<int>(eye_h * scale_y));
			}
		}

		void draw_eye(Renderer& r, const int cx, const int cy, const int rx, const int ry)
		{
			r.draw_ellipse_filled(Vec2(cx, cy), rx, ry, {0, 0, 0, 255});
			r.draw_ellipse_filled(Vec2(cx + rx / 4, cy - ry / 3), rx / 3, ry / 4, {255, 255, 255, 255});
		}
	};

} // namespace robotick
