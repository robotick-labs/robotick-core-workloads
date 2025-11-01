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
		int get_sample_rate() const { return sampleRate; }
		size_t get_frame_count() const { return left_samples.size(); }
		float get_duration_seconds() const;

		const HeapVector<float>& get_left_samples() const { return left_samples; }
		const HeapVector<float>& get_right_samples() const { return right_samples; }

	  private:
		HeapVector<float> left_samples;
		HeapVector<float> right_samples;
		int sampleRate = 44100;
	};

} // namespace robotick
