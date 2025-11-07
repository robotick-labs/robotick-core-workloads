// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#include "robotick/systems/auditory/SpeechToText.h"

#include "whisper.h"

#include <cmath>
#include <cstdlib>
#include <cstring>

namespace robotick
{

	SpeechToText::State* SpeechToText::state = nullptr;

	// --------------------------------------
	// Helpers
	// --------------------------------------
	static inline float seconds_from_ns(uint64_t ns)
	{
		return static_cast<float>(ns) * 1e-9f;
	}

	// --------------------------------------
	// Public API
	// --------------------------------------
	void SpeechToText::init()
	{
		if (state != nullptr)
			return;

		state = new State();
		state->stream.write_index.store(0);
		state->stream.sample_rate = speech_to_text::target_sample_rate_hz;
		state->stream.ring_duration = speech_to_text::ring_buffer_duration_sec;
		state->transcript.clear();
		state->whisper_ctx = nullptr;
	}

	void SpeechToText::reset()
	{
		if (state && state->whisper_ctx)
		{
			whisper_free(reinterpret_cast<whisper_context*>(state->whisper_ctx));
			state->whisper_ctx = nullptr;
		}

		delete state;
		state = nullptr;
	}

	void SpeechToText::push_audio(const float* input_pcm, size_t input_sample_count, size_t input_sample_rate_hz, float engine_time_start)
	{
		if (!state || !input_pcm || input_sample_count == 0 || input_sample_rate_hz == 0)
			return;

		const size_t target_rate = state->stream.sample_rate;
		const size_t max_samples = speech_to_text::ring_buffer_num_samples;

		// Simple integer-ratio-like decimation via running time.
		// (Good enough for voice band; you can swap-in a higher-quality resampler later.)
		// We place samples at times t = resampled_index / target_rate.
		// We choose input samples nearest to those resampled instants.
		size_t last_resampled_index = static_cast<size_t>(-1);

		for (size_t i = 0; i < input_sample_count; ++i)
		{
			const float source_time_sec = static_cast<float>(i) / static_cast<float>(input_sample_rate_hz);
			const size_t resampled_index = static_cast<size_t>(std::round(source_time_sec * static_cast<float>(target_rate)));
			if (resampled_index == last_resampled_index)
				continue;

			last_resampled_index = resampled_index;

			const size_t ring_index = state->stream.write_index.fetch_add(1) % max_samples;
			state->stream.samples[ring_index] = input_pcm[i];
			state->stream.timestamps[ring_index] = engine_time_start + static_cast<float>(resampled_index) / static_cast<float>(target_rate);
		}
	}

	bool SpeechToText::get_audio_window(
		const SpeechToTextBuffer& buffer, const float*& out_samples, size_t& out_sample_count, size_t& out_sample_rate_hz)
	{
		if (!state)
			return false;

		const auto& stream = state->stream;
		const size_t max_count = speech_to_text::ring_buffer_num_samples;

		// NOTE: This linear scan assumes the requested window does not span the ring wrap.
		// That keeps it zero-copy and returns a single contiguous pointer.
		size_t start_index = 0;
		size_t end_index = 0;
		bool found_start = false;
		bool found_end = false;

		for (size_t i = 0; i < max_count; ++i)
		{
			const float t = stream.timestamps[i];
			if (!found_start && t >= buffer.start_time)
			{
				start_index = i;
				found_start = true;
			}
			if (!found_end && t >= buffer.end_time)
			{
				end_index = i;
				found_end = true;
				break;
			}
		}

		if (!found_start || !found_end || end_index <= start_index)
			return false;

		out_samples = &stream.samples[start_index];
		out_sample_count = end_index - start_index;
		out_sample_rate_hz = stream.sample_rate;
		return true;
	}

	const FixedVector<TranscribedWord, 64>& SpeechToText::transcript()
	{
		return state->transcript;
	}

	// --------------------------------------
	// Whisper.cpp Integration
	// --------------------------------------
	bool SpeechToText::load_whisper_model(const char* model_path)
	{
		if (!state)
			init();

		if (state->whisper_ctx)
		{
			whisper_free(reinterpret_cast<whisper_context*>(state->whisper_ctx));
			state->whisper_ctx = nullptr;
		}

		whisper_context_params params = whisper_context_default_params();
		whisper_context* ctx = whisper_init_from_file_with_params(model_path, params);
		if (!ctx)
			return false;

		state->whisper_ctx = ctx;
		return true;
	}

	bool SpeechToText::transcribe_window(
		const SpeechToTextBuffer& buffer, FixedVector<TranscribedWord, 64>& out_words, const char* language, int max_threads)
	{
		if (!state || !state->whisper_ctx)
			return false;

		const float* samples = nullptr;
		size_t count = 0;
		size_t rate = 0;
		if (!get_audio_window(buffer, samples, count, rate))
			return false;

		// whisper expects 16kHz PCM float [-1,1] — we already store at 16k
		// Configure whisper
		whisper_full_params params = whisper_full_default_params(WHISPER_SAMPLING_GREEDY);
		params.print_realtime = false;
		params.print_progress = false;
		params.print_timestamps = false;
		params.print_special = false;
		params.translate = false;	  // no translation; pure STT
		params.no_timestamps = false; // we want per-token times
		params.n_threads = (max_threads > 0) ? max_threads : 1;
		if (language != nullptr)
		{
			params.language = language;
		}

		// Run model
		int rc = whisper_full(reinterpret_cast<whisper_context*>(state->whisper_ctx), params, samples, static_cast<int>(count));
		if (rc != 0)
			return false;

		// Convert tokens to TranscribedWord with engine-time stamps.
		// Strategy: for each segment, read tokens; for each token, compute mid-time
		// and emit 1 "word" per token (filtering out whitespace). For many use-cases,
		// this is sufficiently granular and deterministic.
		out_words.clear();

		const int n_segments = whisper_full_n_segments(reinterpret_cast<whisper_context*>(state->whisper_ctx));
		for (int seg_i = 0; seg_i < n_segments; ++seg_i)
		{
			const int n_tokens = whisper_full_n_tokens(reinterpret_cast<whisper_context*>(state->whisper_ctx), seg_i);
			for (int tok_i = 0; tok_i < n_tokens; ++tok_i)
			{
				const whisper_token_data tdata = whisper_full_get_token_data(reinterpret_cast<whisper_context*>(state->whisper_ctx), seg_i, tok_i);

				const char* token_text = whisper_token_to_str(reinterpret_cast<whisper_context*>(state->whisper_ctx), tdata.id);
				if (!token_text || token_text[0] == '\0')
					continue;

				// Filter out pure whitespace
				bool all_space = true;
				for (const char* p = token_text; *p; ++p)
				{
					if (!std::isspace(static_cast<unsigned char>(*p)))
					{
						all_space = false;
						break;
					}
				}
				if (all_space)
					continue;

				// Token timing is in 10 ms units; convert to seconds relative to this window.
				// t0/t1 can be -1 for special tokens; skip those.
				if (tdata.t0 >= 0 && tdata.t1 >= 0)
				{
					const float token_mid_s = (0.005f * (static_cast<float>(tdata.t0 + tdata.t1) * 0.5f));
					const float engine_time = buffer.start_time + token_mid_s;

					TranscribedWord w{};
					w.engine_time = engine_time;
					w.word = token_text;
					if (out_words.full() == false)
					{
						out_words.add(w);
					}
				}
			}
		}
		return out_words.size() > 0;
	}

} // namespace robotick
