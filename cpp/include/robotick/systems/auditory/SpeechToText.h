// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "robotick/framework/common/FixedString.h"
#include "robotick/framework/common/FixedVector.h"

#include "whisper.h"

#include <atomic>
#include <cstddef>
#include <cstdint>

namespace robotick
{
	struct TranscribedWord
	{
		FixedString32 text;

		float start_time_sec = 0.0f;
		float end_time_sec = 0.0f;
	};

	using TranscribedWords = FixedVector<TranscribedWord, 64>;

	struct SpeechToTextSettings
	{
		FixedString256 model_path;
		uint16_t num_threads = 4;

		float silence_hangover_sec = 0.1f; // how long after voicer is no longer detected, to request a transcribe
	};

	struct SpeechToTextInternalState
	{
		whisper_context_params whisper_cparams;
		whisper_full_params whisper_params;

		whisper_context* whisper_ctx = nullptr;
	};

	struct SpeechToText
	{
		static void initialize(const SpeechToTextSettings& config, SpeechToTextInternalState& state);
		static void uninitialize(SpeechToTextInternalState& state);

		static bool transcribe(const SpeechToTextInternalState& state, const float* buffer, const size_t num_samples, TranscribedWords& out_words);
	};

} // namespace robotick
