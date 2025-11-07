// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstddef>
#include <cstdint>

namespace robotick
{
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

		// --- Output: convenience APIs ---
		// Back-compat: queue a mono buffer. If output is stereo, duplicates to L+R.
		static void write(const float* mono_samples, size_t frames);

		// Queue an interleaved stereo buffer (LRLR...) with 'frames' frames.
		static void write_interleaved_stereo(const float* interleaved_lr, size_t frames);

		// Queue separate left/right mono buffers (will interleave internally).
		static void write_stereo(const float* left, const float* right, size_t frames);

		// Queue a mono buffer into a specific channel (0=left, 1=right). Other channel is zero.
		static void write_mono_to_channel(int channel, const float* mono, size_t frames);

		// --- Input ---
		// Read mono float32 samples from the microphone.
		static size_t read(float* buffer, size_t max_count);
	};

} // namespace robotick
