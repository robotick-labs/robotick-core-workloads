// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#include "robotick/systems/audio/AudioFrame.h"
#include "robotick/framework/registry/TypeDescriptor.h"
#include "robotick/framework/registry/TypeMacros.h"
#include "robotick/framework/registry/TypeRegistry.h"

namespace robotick
{
	ROBOTICK_REGISTER_PRIMITIVE(AudioBuffer128);

	ROBOTICK_REGISTER_STRUCT_BEGIN(AudioBuffer512)
	ROBOTICK_STRUCT_FIXED_ARRAY_FIELD(AudioBuffer512, float, 512, data_buffer)
	ROBOTICK_STRUCT_FIELD(AudioBuffer512, uint32_t, count)
	ROBOTICK_REGISTER_STRUCT_END(AudioBuffer512)

	ROBOTICK_REGISTER_STRUCT_BEGIN(AudioFrame)
	ROBOTICK_STRUCT_FIELD(AudioFrame, AudioBuffer512, samples)
	ROBOTICK_STRUCT_FIELD(AudioFrame, double, timestamp)
	ROBOTICK_STRUCT_FIELD(AudioFrame, uint32_t, sample_rate)
	ROBOTICK_REGISTER_STRUCT_END(AudioFrame)

} // namespace robotick
