// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#include "robotick/systems/audio/AudioFrame.h"
#include "robotick/framework/registry/TypeDescriptor.h"
#include "robotick/framework/registry/TypeMacros.h"
#include "robotick/framework/registry/TypeRegistry.h"

namespace robotick
{

	template <typename T> static bool audio_buffer_to_string(const void* data, char* out_buffer, size_t buffer_size)
	{
		const T* buf = static_cast<const T*>(data);
		if (!buf || !out_buffer || buffer_size < 32)
			return false;

		// Format: <AudioBuffer{capacity}(size)>
		int written = snprintf(out_buffer, buffer_size, "<AudioBuffer%zu(%zu)>", buf->capacity(), buf->size());
		return written > 0 && static_cast<size_t>(written) < buffer_size;
	}

	static bool audio_buffer_from_string(const char*, void*)
	{
		// Read-only string representation, parsing not supported
		return false;
	}

	ROBOTICK_REGISTER_PRIMITIVE(AudioBuffer128, audio_buffer_to_string<AudioBuffer128>, audio_buffer_from_string);
	ROBOTICK_REGISTER_PRIMITIVE(AudioBuffer512, audio_buffer_to_string<AudioBuffer512>, audio_buffer_from_string);

	ROBOTICK_REGISTER_STRUCT_BEGIN(AudioFrame)
	ROBOTICK_STRUCT_FIELD(AudioFrame, AudioBuffer512, samples)
	ROBOTICK_STRUCT_FIELD(AudioFrame, double, timestamp)
	ROBOTICK_STRUCT_FIELD(AudioFrame, uint32_t, sample_rate)
	ROBOTICK_REGISTER_STRUCT_END(AudioFrame)

} // namespace robotick
