// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#include "robotick/systems/Image.h"

#include "robotick/api.h"

namespace robotick
{
	ROBOTICK_REGISTER_PRIMITIVE_WITH_MIME_TYPE(ImageJpegByte, "image/jpeg");
	ROBOTICK_REGISTER_FIXED_VECTOR(ImageJpeg128k, ImageJpegByte);

	ROBOTICK_REGISTER_PRIMITIVE_WITH_MIME_TYPE(ImagePngByte, "image/png");
	ROBOTICK_REGISTER_FIXED_VECTOR(ImagePng16k, ImagePngByte);
	ROBOTICK_REGISTER_FIXED_VECTOR(ImagePng64k, ImagePngByte);
	ROBOTICK_REGISTER_FIXED_VECTOR(ImagePng128k, ImagePngByte);
	ROBOTICK_REGISTER_FIXED_VECTOR(ImagePng256k, ImagePngByte);

} // namespace robotick
