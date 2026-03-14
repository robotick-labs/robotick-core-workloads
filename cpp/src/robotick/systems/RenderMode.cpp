// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#include "robotick/api.h"
#include "robotick/systems/Renderer.h"

namespace robotick
{
	ROBOTICK_REGISTER_ENUM_BEGIN(RenderMode)
	ROBOTICK_ENUM_VALUE("Texture", RenderMode::Texture)
	ROBOTICK_ENUM_VALUE("Screen", RenderMode::Screen)
	ROBOTICK_REGISTER_ENUM_END(RenderMode)
} // namespace robotick
