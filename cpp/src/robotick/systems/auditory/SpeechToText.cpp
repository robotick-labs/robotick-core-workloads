// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#include "robotick/systems/auditory/SpeechToText.h"
#include "robotick/api.h"

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <thread>

namespace robotick
{
	ROBOTICK_REGISTER_STRUCT_BEGIN(SpeechToTextSettings)
	ROBOTICK_STRUCT_FIELD(SpeechToTextSettings, FixedString256, model_path)
	ROBOTICK_STRUCT_FIELD(SpeechToTextSettings, uint16_t, num_threads)
	ROBOTICK_REGISTER_STRUCT_END(SpeechToTextSettings)

	ROBOTICK_REGISTER_STRUCT_BEGIN(TranscribedWord)
	ROBOTICK_STRUCT_FIELD(TranscribedWord, FixedString32, text)
	ROBOTICK_STRUCT_FIELD(TranscribedWord, float, start_time_sec)
	ROBOTICK_STRUCT_FIELD(TranscribedWord, float, end_time_sec)
	ROBOTICK_REGISTER_STRUCT_END(TranscribedWord)

	static bool transcribed_words_to_string(const void* data, char* out_buffer, size_t buffer_size)
	{
		const TranscribedWords* buf = static_cast<const TranscribedWords*>(data);
		if (!buf || !out_buffer || buffer_size < 32)
			return false;

		// Format: <TranscribedWords(size/capacity)>
		int written = snprintf(out_buffer, buffer_size, "<TranscribedWords(%zu/%zu)>", buf->size(), buf->capacity());
		return written > 0 && static_cast<size_t>(written) < buffer_size;
	}

	static bool transcribed_words_from_string(const char*, void*)
	{
		// Read-only string representation, parsing not supported
		return false;
	}

	ROBOTICK_REGISTER_PRIMITIVE(TranscribedWords, transcribed_words_to_string, transcribed_words_from_string);

	void SpeechToText::initialize(const SpeechToTextSettings& settings, SpeechToTextInternalState& state)
	{
		const char* model_path = settings.model_path.c_str();

		// --- Init Whisper context (like CLI) ---
		whisper_context_params& cparams = state.whisper_cparams;
		cparams = whisper_context_default_params();
		cparams.use_gpu = true;	   // CLI tries GPU backend first; harmless if none
		cparams.flash_attn = true; // matches your fast path

		state.whisper_ctx = whisper_init_from_file_with_params(model_path, cparams);
		ROBOTICK_ASSERT(state.whisper_ctx != nullptr);

		// --- Full params: mirror CLI defaults (beam=5, best_of=5 seen in your log) ---
		whisper_full_params& wparams = state.whisper_params;
		wparams = whisper_full_default_params(WHISPER_SAMPLING_BEAM_SEARCH);
		wparams.n_threads = clamp<int>(settings.num_threads, 1, std::thread::hardware_concurrency());
		wparams.offset_ms = 0;
		wparams.duration_ms = 0;
		wparams.translate = false;
		wparams.single_segment = false; // let the lib segment naturally
		wparams.no_context = false;
		wparams.no_timestamps = false; // we want the timestamp of each token wrt the start of the input-waveform
		wparams.max_tokens = 0;
		wparams.print_progress = false;
		wparams.print_realtime = false;
		wparams.print_timestamps = false;
		wparams.print_special = false;

		// hygiene flags
		wparams.suppress_blank = true;
		wparams.suppress_nst = true;
		wparams.temperature = 0.0f;
		wparams.max_initial_ts = 1.0f;
		wparams.length_penalty = -1.0f;
		wparams.temperature_inc = 0.2f;
		wparams.entropy_thold = 2.4f;
		wparams.logprob_thold = -1.0f;
		wparams.no_speech_thold = 0.6f;

		// Decoding knobs matching
		wparams.greedy.best_of = 5;
		wparams.beam_search.beam_size = 5;
		wparams.token_timestamps = false;
		wparams.debug_mode = false;
		wparams.audio_ctx = 0;
		wparams.token_timestamps = true;

		wparams.language = "en";
		wparams.detect_language = false;

		ROBOTICK_INFO(" Initializing Speech to Text - System Info: n_threads = %d / %d | %s\n",
			wparams.n_threads,
			(int)std::thread::hardware_concurrency(),
			whisper_print_system_info());
	}

	bool SpeechToText::transcribe(const SpeechToTextInternalState& state, const float* buffer, size_t num_samples, TranscribedWords& out_words)
	{
		out_words.clear();

		// create a temporary clean state for this inference (so we don't keep accumulating history between calls)
		struct whisper_state* st = whisper_init_state(state.whisper_ctx);

		const bool success = whisper_full_with_state(state.whisper_ctx, st, state.whisper_params, buffer, static_cast<int>(num_samples)) == 0;
		if (!success)
		{
			ROBOTICK_WARNING("SpeechToText - whisper failed to transcribe");
			whisper_free_state(st);
			return false;
		}

		const int num_segments = whisper_full_n_segments_from_state(st);
		for (int seg = 0; seg < num_segments; ++seg)
		{
			const int num_tokens = whisper_full_n_tokens_from_state(st, seg);
			for (int tok = 0; tok < num_tokens; ++tok)
			{
				const whisper_token token = whisper_full_get_token_id_from_state(st, seg, tok);
				const whisper_token_data data = whisper_full_get_token_data_from_state(st, seg, tok);
				const char* text = whisper_token_to_str(state.whisper_ctx, token);
				if (data.t0 >= 0 && data.t1 >= data.t0)
				{
					out_words.add({text, 0.01f * data.t0, 0.01f * data.t1, data.p});
				}
			}
		}

		whisper_free_state(st);
		return true;
	}

	void SpeechToText::uninitialize(SpeechToTextInternalState& state)
	{
		if (state.whisper_ctx)
		{
			whisper_free(state.whisper_ctx);
			state.whisper_ctx = nullptr;
		}
	}

} // namespace robotick
