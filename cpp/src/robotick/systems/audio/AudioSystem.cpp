// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#if defined(ROBOTICK_PLATFORM_DESKTOP) || defined(ROBOTICK_PLATFORM_LINUX)

#include "robotick/systems/audio/AudioSystem.h"

#include "robotick/api.h"
#include "robotick/framework/concurrency/Sync.h"
#include "robotick/framework/containers/HeapVector.h"

#include <SDL2/SDL.h>
#include <cstdint>
#include <cstring>

namespace robotick
{
	ROBOTICK_REGISTER_STRUCT_BEGIN(AudioBackpressureStats)
	ROBOTICK_STRUCT_FIELD(AudioBackpressureStats, uint32_t, drop_events)
	ROBOTICK_STRUCT_FIELD(AudioBackpressureStats, float, dropped_ms)
	ROBOTICK_REGISTER_STRUCT_END(AudioBackpressureStats)

	static constexpr size_t kScratchChunkFrames = 2048;

	class AudioSystemImpl
	{
	  public:
		bool initialized = false;
		bool owns_sdl_audio = false;
		SDL_AudioDeviceID output_device = 0;
		SDL_AudioDeviceID input_device = 0;
		SDL_AudioSpec obtained_output_spec{};
		SDL_AudioSpec obtained_input_spec{};

		HeapVector<float> stereo_scratch;
		HeapVector<float> mono_scratch;
		uint32_t max_queued_bytes = 0;
		AudioBackpressureStrategy strategy = AudioBackpressureStrategy::DropNewest;
		AudioBackpressureStats stats{};

		void cleanup()
		{
			if (output_device != 0)
			{
				SDL_CloseAudioDevice(output_device);
				output_device = 0;
			}
			if (input_device != 0)
			{
				SDL_CloseAudioDevice(input_device);
				input_device = 0;
			}

			::memset(&obtained_output_spec, 0, sizeof(obtained_output_spec));
			::memset(&obtained_input_spec, 0, sizeof(obtained_input_spec));

			if (owns_sdl_audio)
			{
				SDL_QuitSubSystem(SDL_INIT_AUDIO);
				owns_sdl_audio = false;
			}

			initialized = false;
		}

		bool init()
		{
			if (initialized)
				return true;

			if ((SDL_WasInit(SDL_INIT_AUDIO) & SDL_INIT_AUDIO) == 0)
			{
				if (SDL_InitSubSystem(SDL_INIT_AUDIO) < 0)
					return false;
				owns_sdl_audio = true;
			}

			if (!open_devices())
			{
				cleanup();
				return false;
			}

			const size_t stereo_samples = kScratchChunkFrames * 2;
			if (stereo_scratch.size() == 0)
				stereo_scratch.initialize(stereo_samples);
			if (mono_scratch.size() == 0)
				mono_scratch.initialize(kScratchChunkFrames);

			initialized = true;
			return true;
		}

		bool open_devices()
		{
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

			const double queue_seconds = (obtained_output_spec.freq > 0) ? 1.5 : 0.0;
			const double bytes_per_second = static_cast<double>(obtained_output_spec.freq * obtained_output_spec.channels * sizeof(float));
			const double max_bytes = queue_seconds * bytes_per_second;
			if (max_bytes > 0.0 && max_bytes < static_cast<double>(UINT32_MAX))
			{
				max_queued_bytes = static_cast<uint32_t>(max_bytes);
			}
			else
			{
				max_queued_bytes = 0;
			}

			return true;
		}

		uint32_t sample_rate() const { return obtained_output_spec.freq; }
		uint8_t output_channels() const { return obtained_output_spec.channels; }
		uint32_t input_sample_rate() const { return obtained_input_spec.freq != 0 ? obtained_input_spec.freq : obtained_output_spec.freq; }
		uint8_t input_channels() const { return obtained_input_spec.channels != 0 ? obtained_input_spec.channels : 1; }

		// Queue already-interleaved stereo frames (frames == number of LR pairs)
		AudioQueueResult queue_audio_data(const void* data, uint32_t bytes)
		{
			if (output_device == 0 || data == nullptr || bytes == 0)
				return AudioQueueResult::Error;

			const uint32_t queued_bytes = SDL_GetQueuedAudioSize(output_device);
			if (max_queued_bytes != 0 && queued_bytes + bytes > max_queued_bytes)
			{
				const float queued_ms = bytes_to_ms(queued_bytes);
				const float drop_ms = bytes_to_ms(bytes);
				if (strategy == AudioBackpressureStrategy::DropOldest && queued_bytes > 0)
				{
					SDL_ClearQueuedAudio(output_device);
					record_drop(queued_bytes);
					const float cleared_ms = bytes_to_ms(queued_bytes);
					ROBOTICK_WARNING("Audio queue overloaded (%.0fms queued); dropping %.0fms of queued audio to make room", queued_ms, cleared_ms);
					const uint32_t now_queued = SDL_GetQueuedAudioSize(output_device);
					if (max_queued_bytes != 0 && now_queued + bytes > max_queued_bytes)
					{
						record_drop(bytes);
						return AudioQueueResult::Dropped;
					}
				}
				else
				{
					record_drop(bytes);
					ROBOTICK_WARNING("Audio queue overloaded (%.0fms queued); dropping %.0fms of audio", queued_ms, drop_ms);
					return AudioQueueResult::Dropped;
				}
			}

			if (SDL_QueueAudio(output_device, data, bytes) < 0)
			{
				ROBOTICK_WARNING("SDL_QueueAudio failed: %s", SDL_GetError());
				SDL_ClearError();
				return AudioQueueResult::Error;
			}
			return AudioQueueResult::Success;
		}

		float bytes_to_ms(uint32_t bytes) const
		{
			if (obtained_output_spec.freq == 0 || obtained_output_spec.channels == 0)
				return 0.0f;

			const float frame_bytes = static_cast<float>(obtained_output_spec.channels * sizeof(float));
			return (bytes / frame_bytes) / static_cast<float>(obtained_output_spec.freq) * 1000.0f;
		}

		void record_drop(uint32_t bytes)
		{
			stats.drop_events++;
			stats.dropped_ms += bytes_to_ms(bytes);
		}

		AudioQueueResult write_interleaved_stereo(const float* interleaved_lr, size_t frames)
		{
			if (output_device == 0 || interleaved_lr == nullptr || frames == 0)
				return AudioQueueResult::Error;

			if (obtained_output_spec.channels == 1)
			{
				float* mixed = mono_scratch.data();
				size_t remaining = frames;
				const size_t chunk_limit = kScratchChunkFrames;
				const float* src = interleaved_lr;
				while (remaining > 0)
				{
					const size_t chunk = (remaining > chunk_limit) ? chunk_limit : remaining;
					for (size_t i = 0; i < chunk; ++i)
					{
						const float l = src[2 * i + 0];
						const float r = src[2 * i + 1];
						mixed[i] = 0.5f * (l + r);
					}
					const auto result = queue_audio_data(mixed, static_cast<uint32_t>(chunk * sizeof(float)));
					if (result != AudioQueueResult::Success)
						return result;
					src += chunk * 2;
					remaining -= chunk;
				}
				return AudioQueueResult::Success;
			}

			const uint32_t bytes = static_cast<uint32_t>(frames * obtained_output_spec.channels * sizeof(float));
			return queue_audio_data(interleaved_lr, bytes);
		}

		// Queue mono to both channels (duplicates to L+R if output is stereo)
		AudioQueueResult write_mono(const float* mono, size_t frames)
		{
			if (output_device == 0 || mono == nullptr || frames == 0)
				return AudioQueueResult::Error;

			if (obtained_output_spec.channels == 1)
				return queue_audio_data(mono, static_cast<uint32_t>(frames * sizeof(float)));

			size_t remaining = frames;
			const size_t chunk_limit = kScratchChunkFrames;
			float* scratch = stereo_scratch.data();
			while (remaining > 0)
			{
				const size_t chunk = (remaining > chunk_limit) ? chunk_limit : remaining;
				for (size_t i = 0; i < chunk; ++i)
				{
					const float v = mono[i];
					scratch[2 * i + 0] = v;
					scratch[2 * i + 1] = v;
				}
				const auto result = queue_audio_data(scratch, static_cast<uint32_t>(chunk * 2 * sizeof(float)));
				if (result != AudioQueueResult::Success)
					return result;
				mono += chunk;
				remaining -= chunk;
			}
			return AudioQueueResult::Success;
		}

		// Queue mono into a specific channel (0=left, 1=right). Other channel is zero.
		AudioQueueResult write_mono_to_channel(int channel, const float* mono, size_t frames)
		{
			if (output_device == 0 || mono == nullptr || frames == 0)
				return AudioQueueResult::Error;

			if (obtained_output_spec.channels == 1)
				return queue_audio_data(mono, static_cast<uint32_t>(frames * sizeof(float)));

			const int ch = (channel <= 0) ? 0 : 1; // clamp to 0/1
			float* scratch = stereo_scratch.data();
			size_t remaining = frames;
			const size_t chunk_limit = kScratchChunkFrames;

			while (remaining > 0)
			{
				const size_t chunk = (remaining > chunk_limit) ? chunk_limit : remaining;
				for (size_t i = 0; i < chunk; ++i)
				{
					scratch[2 * i + 0] = (ch == 0) ? mono[i] : 0.0f;
					scratch[2 * i + 1] = (ch == 1) ? mono[i] : 0.0f;
				}
				const auto result = queue_audio_data(scratch, static_cast<uint32_t>(chunk * 2 * sizeof(float)));
				if (result != AudioQueueResult::Success)
					return result;
				mono += chunk;
				remaining -= chunk;
			}
			return AudioQueueResult::Success;
		}

		// Queue separate left/right mono buffers
		AudioQueueResult write_stereo(const float* left, const float* right, size_t frames)
		{
			if (output_device == 0 || frames == 0 || (!left && !right))
				return AudioQueueResult::Error;

			if (obtained_output_spec.channels == 1)
			{
				// Mix down to mono (simple average)
				float* mixed = mono_scratch.data();
				size_t remaining = frames;
				const size_t chunk_limit = kScratchChunkFrames;
				while (remaining > 0)
				{
					const size_t chunk = (remaining > chunk_limit) ? chunk_limit : remaining;
					for (size_t i = 0; i < chunk; ++i)
					{
						const float l = left ? left[i] : 0.0f;
						const float r = right ? right[i] : 0.0f;
						mixed[i] = 0.5f * (l + r);
					}
					const auto result = queue_audio_data(mixed, static_cast<uint32_t>(chunk * sizeof(float)));
					if (result != AudioQueueResult::Success)
						return result;
					if (left)
						left += chunk;
					if (right)
						right += chunk;
					remaining -= chunk;
				}
				return AudioQueueResult::Success;
			}

			float* scratch = stereo_scratch.data();
			size_t remaining = frames;
			const size_t chunk_limit = kScratchChunkFrames;
			while (remaining > 0)
			{
				const size_t chunk = (remaining > chunk_limit) ? chunk_limit : remaining;
				for (size_t i = 0; i < chunk; ++i)
				{
					const float l = left ? left[i] : 0.0f;
					const float r = right ? right[i] : 0.0f;
					scratch[2 * i + 0] = l;
					scratch[2 * i + 1] = r;
				}
				const auto result = queue_audio_data(scratch, static_cast<uint32_t>(chunk * 2 * sizeof(float)));
				if (result != AudioQueueResult::Success)
					return result;
				if (left)
					left += chunk;
				if (right)
					right += chunk;
				remaining -= chunk;
			}
			return AudioQueueResult::Success;
		}

		size_t read(float* buffer, size_t max_count)
		{
			if (input_device == 0 || buffer == nullptr || max_count == 0)
				return 0;

			const uint32_t requested_bytes = static_cast<uint32_t>(max_count * sizeof(float));
			const uint32_t dequeued_bytes = SDL_DequeueAudio(input_device, buffer, requested_bytes);

			if (dequeued_bytes == 0)
			{
				const char* err = SDL_GetError();
				if (err != nullptr && err[0] != '\0')
				{
					ROBOTICK_WARNING("AudioSystem::read - SDL_DequeueAudio returned 0 bytes: %s", err);
					SDL_ClearError();
				}
				return 0;
			}

			if ((dequeued_bytes % sizeof(float)) != 0)
			{
				ROBOTICK_WARNING("AudioSystem::read received a partial sample block (%u bytes)", dequeued_bytes);
			}

			return dequeued_bytes / sizeof(float);
		}
	};

	static AudioSystemImpl g_audio_impl;
	static Mutex g_audio_mutex;

	bool AudioSystem::init()
	{
		LockGuard lock(g_audio_mutex);
		if (!g_audio_impl.initialized)
			g_audio_impl.init();
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

	uint32_t AudioSystem::get_input_sample_rate()
	{
		return g_audio_impl.input_sample_rate();
	}

	uint8_t AudioSystem::get_input_channels()
	{
		return g_audio_impl.input_channels();
	}

	AudioQueueResult AudioSystem::write(const float* mono_samples, size_t frames)
	{
		return g_audio_impl.write_mono(mono_samples, frames);
	}

	AudioQueueResult AudioSystem::write_interleaved_stereo(const float* interleaved_lr, size_t frames)
	{
		return g_audio_impl.write_interleaved_stereo(interleaved_lr, frames);
	}

	AudioQueueResult AudioSystem::write_stereo(const float* left, const float* right, size_t frames)
	{
		return g_audio_impl.write_stereo(left, right, frames);
	}

	AudioQueueResult AudioSystem::write_mono_to_channel(int channel, const float* mono, size_t frames)
	{
		return g_audio_impl.write_mono_to_channel(channel, mono, frames);
	}

	size_t AudioSystem::read(float* buffer, size_t max_count)
	{
		return g_audio_impl.read(buffer, max_count);
	}

	void AudioSystem::shutdown()
	{
		LockGuard lock(g_audio_mutex);
		g_audio_impl.cleanup();
	}

	void AudioSystem::set_backpressure_strategy(AudioBackpressureStrategy strategy)
	{
		LockGuard lock(g_audio_mutex);
		g_audio_impl.strategy = strategy;
	}

	AudioBackpressureStrategy AudioSystem::get_backpressure_strategy()
	{
		LockGuard lock(g_audio_mutex);
		return g_audio_impl.strategy;
	}

	AudioBackpressureStats AudioSystem::get_backpressure_stats()
	{
		LockGuard lock(g_audio_mutex);
		return g_audio_impl.stats;
	}

	void AudioSystem::reset_backpressure_stats()
	{
		LockGuard lock(g_audio_mutex);
		g_audio_impl.stats = {};
	}

	void AudioSystem::record_drop_for_test(uint32_t bytes)
	{
		LockGuard lock(g_audio_mutex);
		g_audio_impl.record_drop(bytes);
	}

	void AudioSystem::set_output_spec_for_test(uint32_t sample_rate, uint8_t channels)
	{
		LockGuard lock(g_audio_mutex);
		g_audio_impl.obtained_output_spec.freq = static_cast<int>(sample_rate);
		g_audio_impl.obtained_output_spec.channels = static_cast<Uint8>(channels);
	}

} // namespace robotick

#else

#include "robotick/systems/audio/AudioSystem.h"

namespace robotick
{
	bool AudioSystem::init()
	{
		return false;
	}
	void AudioSystem::shutdown()
	{
	}
	uint32_t AudioSystem::get_sample_rate()
	{
		return 0;
	}
	uint8_t AudioSystem::get_output_channels()
	{
		return 0;
	}
	uint32_t AudioSystem::get_input_sample_rate()
	{
		return 0;
	}
	uint8_t AudioSystem::get_input_channels()
	{
		return 0;
	}
	AudioQueueResult AudioSystem::write(const float*, size_t)
	{
		return AudioQueueResult::Error;
	}
	AudioQueueResult AudioSystem::write_interleaved_stereo(const float*, size_t)
	{
		return AudioQueueResult::Error;
	}
	AudioQueueResult AudioSystem::write_stereo(const float*, const float*, size_t)
	{
		return AudioQueueResult::Error;
	}
	AudioQueueResult AudioSystem::write_mono_to_channel(int, const float*, size_t)
	{
		return AudioQueueResult::Error;
	}
	size_t AudioSystem::read(float*, size_t)
	{
		return 0;
	}
	void AudioSystem::set_backpressure_strategy(AudioBackpressureStrategy)
	{
	}
	AudioBackpressureStrategy AudioSystem::get_backpressure_strategy()
	{
		return AudioBackpressureStrategy::DropNewest;
	}
	AudioBackpressureStats AudioSystem::get_backpressure_stats()
	{
		return {};
	}
	void AudioSystem::reset_backpressure_stats()
	{
	}
	void AudioSystem::record_drop_for_test(uint32_t)
	{
	}
	void AudioSystem::set_output_spec_for_test(uint32_t, uint8_t)
	{
	}
} // namespace robotick

#endif
