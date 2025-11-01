// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "robotick/framework/common/FixedVector.h"

namespace robotick
{
	/**
	 * @brief AudioBuffer64 is a fixed-size audio sample buffer used for audio streaming workloads.
	 * It holds up to 64 float samples (mono).
	 */
	using AudioBuffer64 = FixedVector<float, 64>;

} // namespace robotick
