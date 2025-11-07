// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#include "robotick/systems/audio/AudioSystem.h"

#include <SDL2/SDL.h>
#include <cstring>
#include <mutex>
#include <vector>

namespace robotick
{
	class AudioSystemImpl
	{
	  public:
		bool initialized = false;
		SDL_AudioDeviceID output_device = 0;
		SDL_AudioDeviceID input_device = 0;
		SDL_AudioSpec obtained_output_spec{};
		SDL_AudioSpec obtained_input_spec{};

		bool init()
		{
			if (initialized)
				return true;

			if (SDL_Init(SDL_INIT_AUDIO) < 0)
				return false;

			// --- Output device (speaker) ---
			SDL_AudioSpec desired_output{};
			desired_output.freq = 44100;
			desired_output.format = AUDIO_F32SYS;
			desired_output.channels = 2; // stereo output
			desired_output.samples = 256;
			desired_output.callback = nullptr; // queue mode

			output_device = SDL_OpenAudioDevice(nullptr, 0, &desired_output, &obtained_output_spec, SDL_AUDIO_ALLOW_ANY_CHANGE);
			if (output_device == 0)
				return false;

			SDL_PauseAudioDevice(output_device, 0);

			// --- Input device (microphone) ---
			SDL_AudioSpec desired_input{};
			desired_input.freq = 44100;
			desired_input.format = AUDIO_F32SYS;
			desired_input.channels = 1; // keep mic simple/mono for now
			desired_input.samples = 256;
			desired_input.callback = nullptr;

			input_device = SDL_OpenAudioDevice(nullptr, 1, &desired_input, &obtained_input_spec, SDL_AUDIO_ALLOW_ANY_CHANGE);
			if (input_device == 0)
				return false;

			SDL_PauseAudioDevice(input_device, 0);

			initialized = true;
			return true;
		}

		uint32_t sample_rate() const { return obtained_output_spec.freq; }
		uint8_t output_channels() const { return obtained_output_spec.channels; }

		// Queue already-interleaved stereo frames (frames == number of LR pairs)
		void write_interleaved_stereo(const float* interleaved_lr, size_t frames)
		{
			if (output_device == 0 || interleaved_lr == nullptr || frames == 0)
				return;
			const size_t bytes = frames * obtained_output_spec.channels * sizeof(float);
			SDL_QueueAudio(output_device, interleaved_lr, bytes);
		}

		// Queue mono to both channels (duplicates to L+R if output is stereo)
		void write_mono(const float* mono, size_t frames)
		{
			if (output_device == 0 || mono == nullptr || frames == 0)
				return;

			if (obtained_output_spec.channels == 1)
			{
				SDL_QueueAudio(output_device, mono, frames * sizeof(float));
				return;
			}

			// Interleave mono -> stereo (L=R=mono)
			std::vector<float> tmp;
			tmp.resize(frames * 2);
			for (size_t i = 0; i < frames; ++i)
			{
				tmp[2 * i + 0] = mono[i];
				tmp[2 * i + 1] = mono[i];
			}
			SDL_QueueAudio(output_device, tmp.data(), tmp.size() * sizeof(float));
		}

		// Queue mono into a specific channel (0=left, 1=right). Other channel is zero.
		void write_mono_to_channel(int channel, const float* mono, size_t frames)
		{
			if (output_device == 0 || mono == nullptr || frames == 0)
				return;

			if (obtained_output_spec.channels == 1)
			{
				// Single channel device, just queue as-is
				SDL_QueueAudio(output_device, mono, frames * sizeof(float));
				return;
			}

			const int ch = (channel <= 0) ? 0 : 1; // clamp to 0/1
			std::vector<float> tmp;
			tmp.resize(frames * 2);
			for (size_t i = 0; i < frames; ++i)
			{
				tmp[2 * i + 0] = (ch == 0) ? mono[i] : 0.0f;
				tmp[2 * i + 1] = (ch == 1) ? mono[i] : 0.0f;
			}
			SDL_QueueAudio(output_device, tmp.data(), tmp.size() * sizeof(float));
		}

		// Queue separate left/right mono buffers
		void write_stereo(const float* left, const float* right, size_t frames)
		{
			if (output_device == 0 || frames == 0 || (!left && !right))
				return;

			if (obtained_output_spec.channels == 1)
			{
				// Mix down to mono (simple average)
				std::vector<float> mixed(frames);
				for (size_t i = 0; i < frames; ++i)
				{
					const float l = left ? left[i] : 0.0f;
					const float r = right ? right[i] : 0.0f;
					mixed[i] = 0.5f * (l + r);
				}
				SDL_QueueAudio(output_device, mixed.data(), mixed.size() * sizeof(float));
				return;
			}

			std::vector<float> tmp;
			tmp.resize(frames * 2);
			for (size_t i = 0; i < frames; ++i)
			{
				const float l = left ? left[i] : 0.0f;
				const float r = right ? right[i] : 0.0f;
				tmp[2 * i + 0] = l;
				tmp[2 * i + 1] = r;
			}
			SDL_QueueAudio(output_device, tmp.data(), tmp.size() * sizeof(float));
		}

		size_t read(float* buffer, size_t max_count)
		{
			if (input_device == 0 || buffer == nullptr)
				return 0;

			size_t bytes = SDL_DequeueAudio(input_device, buffer, max_count * sizeof(float));
			return bytes / sizeof(float);
		}
	};

	static AudioSystemImpl g_audio_impl;
	static std::once_flag g_audio_init_flag;

	bool AudioSystem::init()
	{
		std::call_once(g_audio_init_flag,
			[]()
			{
				g_audio_impl.init();
			});
		return g_audio_impl.initialized;
	}

	uint32_t AudioSystem::get_sample_rate()
	{
		return g_audio_impl.sample_rate();
	}

	uint8_t AudioSystem::get_output_channels()
	{
		return g_audio_impl.output_channels();
	}

	void AudioSystem::write(const float* mono_samples, size_t frames)
	{
		// Back-compat: treat as mono write duplicated to both channels if stereo
		g_audio_impl.write_mono(mono_samples, frames);
	}

	void AudioSystem::write_interleaved_stereo(const float* interleaved_lr, size_t frames)
	{
		g_audio_impl.write_interleaved_stereo(interleaved_lr, frames);
	}

	void AudioSystem::write_stereo(const float* left, const float* right, size_t frames)
	{
		g_audio_impl.write_stereo(left, right, frames);
	}

	void AudioSystem::write_mono_to_channel(int channel, const float* mono, size_t frames)
	{
		g_audio_impl.write_mono_to_channel(channel, mono, frames);
	}

	size_t AudioSystem::read(float* buffer, size_t max_count)
	{
		return g_audio_impl.read(buffer, max_count);
	}

} // namespace robotick
