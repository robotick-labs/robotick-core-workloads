// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "robotick/framework/common/FixedVector.h"

namespace robotick
{
	/**
	 * @brief AudioBuffer512 is a fixed-size audio sample buffer used for audio streaming workloads.
	 * It holds up to 512 float samples (mono).
	 */
	using AudioBuffer512 = FixedVector<float, 512>;

	struct AudioFrame
	{
		AudioBuffer512 samples;
		double timestamp = 0.0;
		uint32_t sample_rate = 44100;
	};

} // namespace robotick
