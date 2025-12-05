// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "robotick/framework/containers/FixedVector.h"

namespace robotick
{
	/**
	 * @brief AudioBuffer512 is a fixed-size buffer used for audio streaming workloads.
	 * It holds up to 512 float entries (mono).
	 */
	using AudioBuffer512 = FixedVector<float, 512>;

	/**
	 * @brief AudioBuffer128 is a fixed-size buffer used for audio streaming workloads.
	 * It holds up to 128 float entries (mono).
	 */
	using AudioBuffer128 = FixedVector<float, 128>;

	struct AudioFrame
	{
		AudioBuffer512 samples;
		double timestamp = 0.0;
		uint32_t sample_rate = 44100;
	};

} // namespace robotick
