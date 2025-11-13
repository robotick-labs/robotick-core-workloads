// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#include "robotick/systems/Image.h"

#include "robotick/api.h"

namespace robotick
{
	ROBOTICK_REGISTER_PRIMITIVE_WITH_META(ImageJpegByte, "image/jpeg");
	ROBOTICK_REGISTER_FIXED_VECTOR(ImageJpeg128k, ImageJpegByte);

	ROBOTICK_REGISTER_PRIMITIVE_WITH_META(ImagePngByte, "image/png");
	ROBOTICK_REGISTER_FIXED_VECTOR(ImagePng16k, ImagePngByte);

} // namespace robotick
