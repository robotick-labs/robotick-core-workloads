// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#include "robotick/systems/audio/AudioFrame.h"
#include "robotick/framework/registry/TypeDescriptor.h"
#include "robotick/framework/registry/TypeMacros.h"
#include "robotick/framework/registry/TypeRegistry.h"

namespace robotick
{
	ROBOTICK_REGISTER_FIXED_VECTOR(AudioBuffer128, float)
	ROBOTICK_REGISTER_FIXED_VECTOR(AudioBuffer512, float)

	ROBOTICK_REGISTER_STRUCT_BEGIN(AudioFrame)
	ROBOTICK_STRUCT_FIELD(AudioFrame, AudioBuffer512, samples)
	ROBOTICK_STRUCT_FIELD(AudioFrame, double, timestamp)
	ROBOTICK_STRUCT_FIELD(AudioFrame, uint32_t, sample_rate)
	ROBOTICK_REGISTER_STRUCT_END(AudioFrame)

} // namespace robotick
