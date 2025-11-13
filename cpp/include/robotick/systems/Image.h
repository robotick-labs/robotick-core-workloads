// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "robotick/framework/common/FixedVector.h"

#include <stdint.h>

namespace robotick
{
	using ImageJpegByte = uint8_t;
	using ImageJpeg128k = FixedVector<ImageJpegByte, 128 * 1024>;

	using ImagePngByte = uint8_t;
	using ImagePng16k = FixedVector<ImagePngByte, 16 * 1024>;

} // namespace robotick
