// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#include "robotick/api.h"
#include "robotick/systems/auditory/SpeechToText.h"

namespace robotick
{

	struct SpeechToTextConfig
	{
		// No config yet — may include beam width / model path later
	};

	struct SpeechToTextInputs
	{
		SpeechToTextBuffer buffer;
	};

	struct SpeechToTextOutputs
	{
		FixedVector<TranscribedWord, 64> transcript;
	};

	struct SpeechToTextState
	{
		// Placeholder for future whisper.cpp context caching, etc.
	};

	struct SpeechToTextWorkload
	{
		SpeechToTextConfig config;
		SpeechToTextInputs inputs;
		SpeechToTextOutputs outputs;
		State<SpeechToTextState> state;

		void load() { SpeechToText::init(); }

		void tick(const TickInfo& tick_info)
		{
			(void)tick_info;

			const float* audio = nullptr;
			size_t count = 0;
			size_t rate = 0;

			if (!SpeechToText::get_audio_window(inputs.buffer, audio, count, rate))
			{
				outputs.transcript.clear();
				return;
			}

			// TODO: Call whisper.cpp and fill transcript based on real model output
			outputs.transcript.clear();
			outputs.transcript.add({inputs.buffer.start_time + 0.1f, "hello"});
			outputs.transcript.add({inputs.buffer.start_time + 0.6f, "world"});
		}
	};

} // namespace robotick