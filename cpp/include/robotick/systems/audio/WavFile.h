// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "robotick/framework/common/HeapVector.h"

namespace robotick
{

	class WavFile
	{
	  public:
		bool load(const char* path);
		uint32_t get_sample_rate() const { return sample_rate; }
		size_t get_frame_count() const { return frame_count; }
		float get_duration_seconds() const;

		const HeapVector<float>& get_left_samples() const { return left_samples; }
		const HeapVector<float>& get_right_samples() const { return right_samples; }

	  private:
		HeapVector<float> left_samples;
		HeapVector<float> right_samples;
		uint32_t sample_rate = 44100;
		size_t frame_count = 0;
	};

} // namespace robotick
