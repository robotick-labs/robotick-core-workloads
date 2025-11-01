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
	};

	struct WavPlayerOutputs
	{
		AudioBuffer64 left;
		AudioBuffer64 right;

		float total_duration_sec = 0.0f;
		uint64_t total_frame_count = 0;
	};

	struct WavPlayerState
	{
		WavFile wav_file;
		size_t current_frame = 0;
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

		void start(float tick_rate_hz)
		{
			// nothing needed yet
		}

		void tick(const TickInfo& info)
		{
			const WavFile& wav_file = state->wav_file;

			int target_rate = wav_file.get_sample_rate();
			int samples_per_tick = target_rate / static_cast<int>(info.tick_rate_hz);

			outputs.left.set_size(0);
			outputs.right.set_size(0);

			for (int i = 0; i < samples_per_tick; ++i)
			{
				if (state->current_frame >= wav_file.get_left_samples().size())
				{
					if (config.looping)
					{
						state->current_frame = 0;
					}
					else
					{
						break;
					}
				}

				outputs.left.add(wav_file.get_left_samples()[state->current_frame]);
				outputs.right.add(wav_file.get_right_samples()[state->current_frame]);
				state->current_frame++;
			}
		}
	};

} // namespace robotick