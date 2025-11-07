// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "robotick/framework/common/FixedString.h"
#include "robotick/framework/common/FixedVector.h"

#include <atomic>
#include <cstddef>
#include <cstdint>

namespace robotick
{

	namespace speech_to_text
	{
		// Fixed operating parameters for STT ring
		constexpr size_t target_sample_rate_hz = 16000;
		constexpr size_t ring_buffer_duration_sec = 10;
		constexpr size_t ring_buffer_num_samples = target_sample_rate_hz * ring_buffer_duration_sec;
	} // namespace speech_to_text

	// -------------------------------
	// Struct: TranscribedWord
	// -------------------------------
	struct TranscribedWord
	{
		float engine_time = 0.0f; // Engine timeline (sec)
		FixedString32 word;		  // Token/word text
	};

	// -------------------------------
	// Struct: SpeechToTextBuffer
	// -------------------------------
	struct SpeechToTextBuffer
	{
		float start_time = 0.0f; // Engine time (sec)
		float end_time = 0.0f;
	};

	// -------------------------------
	// Class: SpeechToText
	// -------------------------------
	class SpeechToText
	{
	  public:
		// Lifetime
		static void init();
		static void reset();

		// Real-time write path (mono PCM, any input rate). Internally resampled to 16kHz.
		static void push_audio(const float* input_pcm, size_t input_sample_count, size_t input_sample_rate_hz, float engine_time_start);

		// Zero-copy window view into ring buffer at fixed 16kHz.
		// NOTE: Returns a contiguous view only if the time window does not cross the ring wrap.
		static bool get_audio_window(
			const SpeechToTextBuffer& buffer, const float*& out_samples, size_t& out_sample_count, size_t& out_sample_rate_hz);

		// Transcript storage (owned by the STT system; useful for diagnostics/UI).
		static const FixedVector<TranscribedWord, 64>& transcript();

		// -------- Whisper.cpp integration (optional) --------
		// Load a ggml model and keep a context in the STT system.
		// Returns false if whisper is unavailable or model failed to load.
		static bool load_whisper_model(const char* model_path);

		// Transcribe a time window from the ring buffer using whisper.cpp and
		// return timestamped words in out_words. Returns false if model not loaded
		// or the window cannot be obtained.
		static bool transcribe_window(const SpeechToTextBuffer& buffer,
			FixedVector<TranscribedWord, 64>& out_words,
			const char* language /* e.g., "en" */ = nullptr,
			int max_threads = 1);

	  private:
		struct StreamBuffer
		{
			FixedVector<float, speech_to_text::ring_buffer_num_samples> samples;
			FixedVector<float, speech_to_text::ring_buffer_num_samples> timestamps; // per-sample engine time
			std::atomic<size_t> write_index{0};
			size_t sample_rate = speech_to_text::target_sample_rate_hz;
			size_t ring_duration = speech_to_text::ring_buffer_duration_sec;
		};

		struct State
		{
			StreamBuffer stream;
			FixedVector<TranscribedWord, 64> transcript; // optional general store

			// Whisper context (optional; null if not loaded)
			void* whisper_ctx = nullptr; // stored as void* to avoid hard dependency in header
		};

		static State* state;
	};

} // namespace robotick
