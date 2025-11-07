// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#include "robotick/api.h"
#include "robotick/systems/audio/AudioFrame.h"
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

		float amplitude_gain_db = 0.0f; // Linear gain multiplier = pow(10, amplitude_gain_db / 20)

		bool looping = false;
		float loop_delay_sec = 0.0f;
	};

	struct WavPlayerOutputs
	{
		AudioFrame left;
		AudioFrame right;

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

			outputs.left.sample_rate = AudioSystem::get_sample_rate();
			outputs.right.sample_rate = outputs.left.sample_rate;

			outputs.total_duration_sec = wav_file.get_duration_seconds();
			outputs.total_frame_count = wav_file.get_frame_count();

			ROBOTICK_ASSERT_MSG(AudioSystem::get_sample_rate() == wav_file.get_sample_rate(),
				"Audio System sample-rate (%i) and that of wav-file '%s' (%i) differ",
				AudioSystem::get_sample_rate(),
				config.file_path.c_str(),
				(int)wav_file.get_sample_rate());
		}

		void start(float /*tick_rate_hz*/) { state->time_to_loop_sec = config.loop_delay_sec; }

		void tick(const TickInfo& tick_info)
		{
			static constexpr double ns_to_sec = 1e-9;
			outputs.left.timestamp = ns_to_sec * (double)tick_info.time_now_ns;
			outputs.right.timestamp = outputs.left.timestamp;

			const WavFile& wav_file = state->wav_file;

			const size_t frame_count = wav_file.get_frame_count();
			const int target_rate = wav_file.get_sample_rate();
			const int samples_per_tick = target_rate / static_cast<int>(tick_info.tick_rate_hz);

			int remaining = frame_count - static_cast<int>(state->current_frame);
			int emit_samples = std::min(samples_per_tick, remaining);

			if (emit_samples > 0)
			{
				const float* left_ptr = &wav_file.get_left_samples()[state->current_frame];
				const float* right_ptr = &wav_file.get_right_samples()[state->current_frame];

				const float gain = std::pow(10.0f, config.amplitude_gain_db / 20.0f);

				if (gain == 1.0f)
				{
					outputs.left.samples.set(left_ptr, emit_samples);
					outputs.right.samples.set(right_ptr, emit_samples);
				}
				else
				{
					outputs.left.samples.set_size(emit_samples);
					outputs.right.samples.set_size(emit_samples);
					for (int i = 0; i < emit_samples; ++i)
					{
						outputs.left.samples[i] = gain * left_ptr[i];
						outputs.right.samples[i] = gain * right_ptr[i];
					}
				}

				state->current_frame += emit_samples;
			}
			else
			{
				outputs.left.samples.fill(0.0f);
				outputs.right.samples.fill(0.0f);
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