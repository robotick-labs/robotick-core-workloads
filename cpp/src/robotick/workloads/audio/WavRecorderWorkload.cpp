// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#include "robotick/api.h"
#include "robotick/systems/audio/AudioFrame.h"
#include "robotick/systems/audio/AudioSystem.h"
#include "robotick/systems/audio/WavFile.h"

#include <cmath>
#include <cstdint>
#include <string>

namespace robotick
{
	struct WavRecorderConfig
	{
		FixedString256 file_path; // Destination WAV file (16-bit PCM)
		bool overwrite = true;	  // Replace if exists
		bool stereo = false;	  // true = interleave L/R from two inputs, else mono only
	};

	struct WavRecorderInputs
	{
		AudioFrame left;  // required
		AudioFrame right; // optional if stereo
	};

	struct WavRecorderOutputs
	{
		bool file_open = false;
		uint64_t total_written = 0;
	};

	struct WavRecorderState
	{
		WavFileWriter wav_file;
	};

	struct WavRecorderWorkload
	{
		WavRecorderConfig config;
		WavRecorderInputs inputs;
		WavRecorderOutputs outputs;

		State<WavRecorderState> state;

		void load()
		{
			AudioSystem::init();

			// Prepare file
			const char* path = config.file_path.c_str();
			if (!config.overwrite && WavFile::exists(path))
				ROBOTICK_FATAL_EXIT("WAV Recorder: file exists and overwrite=false: %s", path);

			const int fs = AudioSystem::get_sample_rate();
			if (!state->wav_file.open(path, fs, config.stereo ? 2 : 1))
				ROBOTICK_FATAL_EXIT("WAV Recorder: failed to open for writing: %s", path);

			outputs.file_open = true;
			outputs.total_written = 0;
		}

		void tick(const TickInfo&)
		{
			if (!outputs.file_open)
				return;

			const size_t n = inputs.left.samples.size();
			if (n == 0)
				return;

			if (config.stereo)
			{
				const size_t nr = robotick::min(n, inputs.right.samples.size());
				state->wav_file.append_stereo(inputs.left.samples.data(), inputs.right.samples.data(), nr);
				outputs.total_written += nr;
			}
			else
			{
				state->wav_file.append_mono(inputs.left.samples.data(), n);
				outputs.total_written += n;
			}
		}

		void stop()
		{
			if (outputs.file_open)
			{
				state->wav_file.close();
				outputs.file_open = false;
			}
		}
	};

} // namespace robotick
