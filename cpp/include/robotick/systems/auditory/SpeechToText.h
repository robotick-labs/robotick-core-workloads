// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "robotick/framework/containers/FixedVector.h"
#include "robotick/framework/strings/FixedString.h"

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

		float confidence = 0.0f;
	};

	using TranscribedWords = FixedVector<TranscribedWord, 64>;

	struct Transcript
	{
		TranscribedWords words;
		FixedString512 text;

		float transcribe_duration_sec = 0.0f;
		float transcript_mean_confidence = 0.0f;
		float start_time_sec = 0.0f;
		float duration_sec = 0.0f;

		void clear()
		{
			words.clear();
			text.clear();
			transcribe_duration_sec = 0.0f;
			transcript_mean_confidence = 0.0f;
			start_time_sec = 0.0f;
			duration_sec = 0.0f;
		}

		void update_timing_from_words(const float fallback_start_time_sec, const float fallback_duration_sec)
		{
			if (words.empty())
			{
				start_time_sec = fallback_start_time_sec;
				duration_sec = fallback_duration_sec;
				return;
			}

			start_time_sec = words[0].start_time_sec;
			const float end_time_sec = words[words.size() - 1].end_time_sec;
			duration_sec = (end_time_sec >= start_time_sec) ? (end_time_sec - start_time_sec) : fallback_duration_sec;
		}
	};

	struct SpeechToTextSettings
	{
		FixedString256 model_path;
		uint16_t num_threads = 4;

		float min_voiced_duration_sec = 0.5f;
		float silence_hangover_sec = 0.2f; // how long after voice is no longer detected, to request a transcribe
		float proto_refresh_interval_sec = 0.2f;
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
