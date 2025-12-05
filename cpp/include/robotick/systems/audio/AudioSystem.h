// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstddef>
#include <cstdint>

namespace robotick
{
	enum class AudioBackpressureStrategy
	{
		DropNewest,
		DropOldest,
	};

	struct AudioBackpressureStats
	{
		uint32_t drop_events = 0;
		float dropped_ms = 0.0f;
	};

	enum class AudioQueueResult
	{
		Success,
		Dropped,
		Error,
	};

	struct AudioReadResult
	{
		AudioQueueResult status = AudioQueueResult::Error;
		size_t samples_read = 0;
	};

	/**
	 * @brief Singleton audio system wrapper for SDL2
	 *
	 * Provides stereo output (float32), mono mic input, and helpers to write
	 * mono/stereo buffers. All write() calls are non-blocking queue operations.
	 */
	class AudioSystem
	{
	  public:
		// Initialize the audio system (idempotent)
		static bool init();

		// Output device info
		static uint32_t get_sample_rate();
		static uint8_t get_output_channels(); // e.g. 2 for stereo

		// Input device info (microphone)
		static uint32_t get_input_sample_rate();
		static uint8_t get_input_channels();

		// --- Output: convenience APIs ---
		// Queue a mono buffer (duplicates across channels if device is stereo).
		static AudioQueueResult write(const float* mono_samples, size_t frames);

		// Queue an interleaved stereo buffer (LRLR...) with 'frames' frames.
		static AudioQueueResult write_interleaved_stereo(const float* interleaved_lr, size_t frames);

		// Queue separate left/right mono buffers (will interleave internally).
		static AudioQueueResult write_stereo(const float* left, const float* right, size_t frames);

		// Queue a mono buffer into a specific channel (0=left, 1=right). Other channel is zero.
		static AudioQueueResult write_mono_to_channel(int channel, const float* mono, size_t frames);

		// --- Input ---
		// Read mono float32 samples from the microphone.
		static AudioReadResult read(float* buffer, size_t max_count);

		// Shutdown audio devices and release SDL resources.
		static void shutdown();

		// Back-pressure controls
		static void set_backpressure_strategy(AudioBackpressureStrategy strategy);
		static AudioBackpressureStrategy get_backpressure_strategy();
		static AudioBackpressureStats get_backpressure_stats();
		static void reset_backpressure_stats();
		static void record_drop_for_test(uint32_t bytes);
		static void set_output_spec_for_test(uint32_t sample_rate, uint8_t channels);
	};

} // namespace robotick
