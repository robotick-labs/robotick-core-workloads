// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "robotick/framework/common/HeapVector.h"

#include <cstdint>
#include <cstdio>

namespace robotick
{

	class WavFile
	{
	  public:
		// === Load (read-only) ===
		bool load(const char* path);
		uint32_t get_sample_rate() const { return sample_rate; }
		size_t get_frame_count() const { return frame_count; }
		float get_duration_seconds() const;
		uint16_t get_num_channels() const { return num_channels; };

		const HeapVector<float>& get_left_samples() const { return left_samples; }
		const HeapVector<float>& get_right_samples() const { return right_samples; }

		// === Write (record) ===
		bool open_write(const char* path, uint32_t sample_rate, uint16_t num_channels = 1);
		void append_mono(const float* samples, size_t count);
		void append_stereo(const float* left, const float* right, size_t count);
		void close_write();

		static bool exists(const char* path);

	  private:
		// Shared
		uint32_t sample_rate = 44100;
		size_t frame_count = 0;
		uint16_t num_channels = 0;

		// Read buffers
		HeapVector<float> left_samples;
		HeapVector<float> right_samples;

		// Write state
		FILE* fp = nullptr;
		uint16_t write_channels = 0;
		uint32_t data_bytes_written = 0;

		void write_header_placeholder(uint32_t sr, uint16_t ch);
		void patch_header();
	};

} // namespace robotick
