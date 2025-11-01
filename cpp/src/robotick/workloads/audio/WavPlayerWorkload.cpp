// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#include "robotick/api.h"
#include "robotick/systems/audio/AudioBuffer.h"
#include "robotick/systems/audio/AudioSystem.h"
#include "robotick/systems/audio/WavFile.h"

#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

namespace robotick
{
	struct WavPlayerConfig
	{
		FixedString256 file_path; // Path to WAV file (16-bit PCM, stereo)

		bool looping = false;
		float loop_delay_sec = 0.0f;
	};

	struct WavPlayerOutputs
	{
		AudioBuffer512 left;
		AudioBuffer512 right;

		float total_duration_sec = 0.0f;
		uint64_t total_frame_count = 0;
	};

	struct WavPlayerState
	{
		WavFile wav_file;
		size_t current_frame = 0;

		float time_to_loop_sec = 0.0f;
	};

	struct WavPlayerWorkload
	{
		WavPlayerConfig config;
		WavPlayerOutputs outputs;
		State<WavPlayerState> state;

		void load()
		{
			AudioSystem::init();

			WavFile& wav_file = state->wav_file;

			if (!wav_file.load(config.file_path.c_str()))
				ROBOTICK_FATAL_EXIT("Failed to open WAV file: %s", config.file_path.c_str());

			outputs.total_duration_sec = wav_file.get_duration_seconds();
			outputs.total_frame_count = wav_file.get_frame_count();

			ROBOTICK_ASSERT_MSG(AudioSystem::get_sample_rate() == wav_file.get_sample_rate(),
				"Audio System sample-rate (%i) and that of wav-file '%s' (%i) differ",
				AudioSystem::get_sample_rate(),
				config.file_path.c_str(),
				wav_file.get_sample_rate());
		}

		void start(float tick_rate_hz) { state->time_to_loop_sec = config.loop_delay_sec; }

		void tick(const TickInfo& tick_info)
		{
			const WavFile& wav_file = state->wav_file;

			const int frame_count = wav_file.get_frame_count();
			const int target_rate = wav_file.get_sample_rate();
			const int samples_per_tick = target_rate / static_cast<int>(tick_info.tick_rate_hz);

			int remaining = frame_count - static_cast<int>(state->current_frame);
			int emit_samples = std::min(samples_per_tick, remaining);

			if (emit_samples > 0)
			{
				const float* left_ptr = &wav_file.get_left_samples()[state->current_frame];
				const float* right_ptr = &wav_file.get_right_samples()[state->current_frame];

				outputs.left.set(left_ptr, emit_samples);
				outputs.right.set(right_ptr, emit_samples);

				state->current_frame += emit_samples;
			}
			else
			{
				outputs.left.set_size(0);
				outputs.right.set_size(0);
			}

			// Loop if enabled and we're at the end
			if (state->current_frame >= frame_count && config.looping)
			{
				if (state->time_to_loop_sec > 0.0f)
				{
					state->time_to_loop_sec -= tick_info.delta_time;
				}
				else
				{
					state->current_frame = 0;
					state->time_to_loop_sec = config.loop_delay_sec;
				}
			}
		}
	};

} // namespace robotick