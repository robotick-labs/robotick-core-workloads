// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#include "robotick/api.h"
#include "robotick/systems/AudioBuffer.h"

#include <SDL2/SDL.h>
#include <cmath>
#include <mutex>

namespace robotick
{
	struct SpeakerInputs
	{
		AudioBuffer64 samples;
	};

	struct SpeakerState
	{
		SDL_AudioDeviceID audio_device = 0;
		SDL_AudioSpec obtained_spec{};
	};

	struct SpeakerWorkload
	{
		SpeakerInputs inputs;
		State<SpeakerState> state;

		void load()
		{
			static std::once_flag sdl_init_flag;
			std::call_once(sdl_init_flag,
				[]()
				{
					SDL_Init(SDL_INIT_AUDIO);
				});

			SDL_AudioSpec desired{};
			desired.freq = 44100;
			desired.format = AUDIO_F32SYS; // float32 samples
			desired.channels = 1;		   // mono
			desired.samples = 256;		   // buffer size
			desired.callback = nullptr;	   // use queue mode

			state->audio_device = SDL_OpenAudioDevice(nullptr, 0, &desired, &state->obtained_spec, SDL_AUDIO_ALLOW_ANY_CHANGE);
			if (state->audio_device == 0)
			{
				ROBOTICK_FATAL_EXIT("Failed to open audio: %s\n", SDL_GetError());
			}
			else
			{
				SDL_PauseAudioDevice(state->audio_device, 0); // start playback
				ROBOTICK_INFO("Opened SDL audio (%d Hz)\n", state->obtained_spec.freq);
			}
		}

		void tick(const TickInfo&)
		{
			if (state->audio_device == 0)
				return;

			const size_t sample_count = inputs.samples.size();
			const float* samples = inputs.samples.data();

			// Queue audio to SDL
			SDL_QueueAudio(state->audio_device, samples, sample_count * sizeof(float));

			// Debug RMS volume
			float sum_sq = 0.0f;
			for (size_t i = 0; i < sample_count; ++i)
				sum_sq += samples[i] * samples[i];

			float rms = sample_count > 0 ? std::sqrt(sum_sq / sample_count) : 0.0f;
			printf("[Speaker] RMS = %.4f (%zu samples)\n", rms, sample_count);
		}
	};

} // namespace robotick
