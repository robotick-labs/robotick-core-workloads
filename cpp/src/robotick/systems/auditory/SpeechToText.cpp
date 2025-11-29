// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#include "robotick/systems/auditory/SpeechToText.h"

#include "robotick/api.h"
#include "robotick/framework/concurrency/Thread.h"

#include <cmath>
#include <cstdlib>
#include <cstring>

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

	ROBOTICK_REGISTER_FIXED_VECTOR(TranscribedWords, TranscribedWord);

	void whisper_log_handler(enum ggml_log_level level, const char* text, void* user_data)
	{
		(void)user_data;

		if (level == GGML_LOG_LEVEL_ERROR)
		{
			ROBOTICK_WARNING("[WHISPER ERROR] %s", text);
		}
		else if (level == GGML_LOG_LEVEL_WARN)
		{
			ROBOTICK_WARNING("[WHISPER WARN] %s", text);
		}
		// else ignore all other logs
	}

	void SpeechToText::initialize(const SpeechToTextSettings& settings, SpeechToTextInternalState& state)
	{
		// silence all logs but errors and warnings
		whisper_log_set(whisper_log_handler, nullptr);

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
		wparams.n_threads = clamp<int>(settings.num_threads, 1, Thread::get_hardware_concurrency());
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
		wparams.debug_mode = false;
		wparams.audio_ctx = 0;
		wparams.token_timestamps = true;

		wparams.language = "en";
		wparams.detect_language = false;

		ROBOTICK_INFO(" Initializing Speech to Text - System Info: n_threads = %d / %d | %s\n",
			wparams.n_threads,
			(int)Thread::get_hardware_concurrency(),
			whisper_print_system_info());
	}

	bool SpeechToText::transcribe(const SpeechToTextInternalState& state, const float* buffer, size_t num_samples, TranscribedWords& out_words)
	{
		out_words.clear();

		// create a temporary clean state for this inference (so we don't keep accumulating history between calls)
		struct whisper_state* wstate = whisper_init_state(state.whisper_ctx);
		if (!wstate)
		{
			ROBOTICK_WARNING("SpeechToText - failed to allocate whisper state");
			return false;
		}

		const bool success = whisper_full_with_state(state.whisper_ctx, wstate, state.whisper_params, buffer, static_cast<int>(num_samples)) == 0;
		if (!success)
		{
			ROBOTICK_WARNING("SpeechToText - whisper failed to transcribe");
			whisper_free_state(wstate);
			return false;
		}

		bool is_at_capacity = false;
		const int num_segments = whisper_full_n_segments_from_state(wstate);
		for (int seg = 0; seg < num_segments && !is_at_capacity; ++seg)
		{
			const int num_tokens = whisper_full_n_tokens_from_state(wstate, seg);
			for (int tok = 0; tok < num_tokens && !is_at_capacity; ++tok)
			{
				const whisper_token token = whisper_full_get_token_id_from_state(wstate, seg, tok);
				const whisper_token_data data = whisper_full_get_token_data_from_state(wstate, seg, tok);
				const char* text = whisper_token_to_str(state.whisper_ctx, token);
				if (data.t0 >= 0 && data.t1 >= data.t0)
				{
					out_words.add({text, 0.01f * data.t0, 0.01f * data.t1, data.p});
					is_at_capacity = (out_words.size() >= out_words.capacity());
				}
			}
		}

		whisper_free_state(wstate);
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
