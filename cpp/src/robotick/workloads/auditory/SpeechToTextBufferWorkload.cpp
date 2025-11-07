// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#include "robotick/api.h"
#include "robotick/systems/audio/AudioFrame.h"
#include "robotick/systems/auditory/SpeechToText.h"

namespace robotick
{

	struct SpeechToTextBufferConfig
	{
		// No config for now
	};

	struct SpeechToTextBufferInputs
	{
		AudioFrame mono;
	};

	struct SpeechToTextBufferOutputs
	{
		SpeechToTextBuffer buffer;
	};

	struct SpeechToTextBufferState
	{
		// No persistent state
	};

	struct SpeechToTextBufferWorkload
	{
		SpeechToTextBufferConfig config;
		SpeechToTextBufferInputs inputs;
		SpeechToTextBufferOutputs outputs;
		State<SpeechToTextBufferState> state;

		void load() { SpeechToText::init(); }

		void tick(const TickInfo& tick_info)
		{
			SpeechToText::push_audio(inputs.mono.samples.data(), inputs.mono.samples.size(), inputs.mono.sample_rate, tick_info.time_now);

			outputs.buffer.start_time =
				static_cast<float>(tick_info.time_now_ns) * 1e-9f - static_cast<float>(speech_to_text::ring_buffer_duration_sec);
			outputs.buffer.end_time = static_cast<float>(tick_info.time_now_ns) * 1e-9f;
		}
	};

} // namespace robotick