// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#if defined(ROBOTICK_PLATFORM_DESKTOP) || defined(ROBOTICK_PLATFORM_LINUX)

#include "robotick/api.h"
#include "robotick/framework/concurrency/Atomic.h"
#include "robotick/framework/concurrency/Sync.h"
#include "robotick/framework/concurrency/Thread.h"
#include "robotick/framework/time/Clock.h"
#include "robotick/systems/audio/AudioFrame.h"
#include "robotick/systems/auditory/SpeechToText.h"

#include <cstring>

namespace robotick
{
	struct SpeechToTextConfig
	{
		SpeechToTextSettings settings;
	};

	struct SpeechToTextInputs
	{
		AudioFrame mono;
		bool is_voiced = false;
	};

	struct SpeechToTextOutputs
	{
		TranscribedWords words;
		FixedString512 transcript;
		float transcribe_duration_sec = 0.0f;
		float transcript_mean_confidence = 0.0f;

		float accumulator_duration_sec = 0.0f;
		float accumulator_capacity_sec = 0.0f;

		bool is_transcribe_thread_active = false;
		uint32_t transcribe_session_count = 0;
	};

	static constexpr uint32_t accumulator_capacity_sec = 20;
	static constexpr uint32_t accumulator_sample_rate_hz = 16000;

	struct AudioAccumulator
	{
		using AccumulatorSamples = FixedVector<float, accumulator_capacity_sec * accumulator_sample_rate_hz>;

		AccumulatorSamples samples;
		float end_time_sec = 0.0f;

		float get_duration_sec() const { return static_cast<float>(samples.size()) / accumulator_sample_rate_hz; }
		static constexpr float get_capacity_sec() { return static_cast<float>(AccumulatorSamples::capacity()) / accumulator_sample_rate_hz; }

		void request_drop_oldest_duration_sec(const float drop_secs)
		{
			size_t samples_to_drop = static_cast<size_t>(drop_secs * (float)accumulator_sample_rate_hz);
			if (samples_to_drop > samples.size())
			{
				samples_to_drop = samples.size();
			}

			const size_t keep_count = samples.size() - samples_to_drop;
			memmove(samples.data(), samples.data() + samples_to_drop, keep_count * sizeof(float));
			samples.set_size(keep_count);
		}
	};

	struct SpeechToTextState
	{
		SpeechToTextInternalState internal_state;

		AudioAccumulator audio_accumulators[2];
		AtomicFlag is_buffer_swapped{false}; // false → [0] is FG, true → [1] is FG

		TranscribedWords last_result;
		FixedString512 last_transcript;
		float last_transcript_mean_confidence = 0.0f;

		float transcribe_start_time_sec = 0.0f;

		AtomicFlag is_bgthread_active{false};
		AtomicFlag has_new_transcript{false};

		Thread bg_thread;
		Mutex mutex;
		ConditionVariable cv;
		bool thread_should_exit = false;
		bool thread_has_work = false;

		double last_voiced_time = 0.0;
		bool was_voiced_last_tick = false;
		bool is_voiced_segment_pending = false;

		AudioAccumulator& get_foreground_accumulator() { return is_buffer_swapped.is_set() ? audio_accumulators[1] : audio_accumulators[0]; }
		AudioAccumulator& get_background_accumulator() { return is_buffer_swapped.is_set() ? audio_accumulators[0] : audio_accumulators[1]; }
	};

	// ---------------------------------------------------------------
	// Simple linear downsampler to 16 kHz
	// ---------------------------------------------------------------
	static void downsample_to_accumulator_rate(const AudioBuffer512& input, const uint32_t input_rate, AudioBuffer512& output)
	{
		const double ratio = static_cast<double>(input_rate) / static_cast<double>(accumulator_sample_rate_hz);
		if (ratio <= 0.0)
		{
			return;
		}

		const size_t ideal_dst = static_cast<size_t>(static_cast<double>(input.size()) / ratio);
		const size_t dst_count = (ideal_dst < output.capacity()) ? ideal_dst : output.capacity();
		output.clear(); // make sure it's empty, as expected

		for (size_t dst_index = 0; dst_index < dst_count; ++dst_index)
		{
			const double src_index_f = static_cast<double>(dst_index) * ratio;
			const size_t src_index_i = static_cast<size_t>(src_index_f);
			const double frac = src_index_f - static_cast<double>(src_index_i);

			// Clamp hi index to last valid sample to be extra safe on edge rounding.
			const size_t src_next = (src_index_i + 1 < input.size()) ? (src_index_i + 1) : src_index_i;

			const float sample =
				static_cast<float>(static_cast<double>(input[src_index_i]) * (1.0 - frac) + static_cast<double>(input[src_next]) * frac);

			// No need to check size each iteration because we clamp dst_count.
			output.add(sample);
		}
	}

	// ---------------------------------------------------------------
	// Background inference thread
	// ---------------------------------------------------------------
	static void speech_to_text_thread(void* user_data)
	{
		SpeechToTextState* state = static_cast<SpeechToTextState*>(user_data);

		while (true)
		{
			UniqueLock lock(state->mutex);
			state->cv.wait(lock,
				[&]()
				{
					return state->thread_has_work || state->thread_should_exit;
				});

			if (state->thread_should_exit)
			{
				break;
			}

			state->thread_has_work = false;
			lock.unlock();

			state->is_bgthread_active.set();

			const AudioAccumulator& audio_accumulator = state->get_background_accumulator();

			const float audio_accumulator_duration_sec = audio_accumulator.get_duration_sec();
			const float start_time_sec_engine = audio_accumulator.end_time_sec - audio_accumulator_duration_sec;

			if (!audio_accumulator.samples.empty())
			{
				TranscribedWords transcribed_words_raw;
				TranscribedWords transcribed_words;
				FixedString512 transcript;

				SpeechToText::transcribe(
					state->internal_state, audio_accumulator.samples.data(), audio_accumulator.samples.size(), transcribed_words_raw);

				float mean_confidence = 0.0f;

				for (TranscribedWord& word : transcribed_words_raw)
				{
					mean_confidence += word.confidence / (float)transcribed_words_raw.size();

					if (word.text.contains('['))
					{
						continue; // skip tokens
					}

					word.start_time_sec += start_time_sec_engine;
					word.end_time_sec += start_time_sec_engine;

					// Remove any preceding space from first word (for tidiness):
					if (transcribed_words.size() == 0)
					{
						const char* original_chars = word.text.c_str();
						if (original_chars[0] == ' ')
						{
							const FixedString32 swap_string = &original_chars[1];
							// ^- assumes every string will have a null-terminator - which is the case for FixedString
							word.text = swap_string;
						}
					}

					transcribed_words.add(word);
					transcript.append(word.text.c_str());
				}

				lock.lock();
				state->last_result = transcribed_words;
				state->last_transcript = transcript;
				state->last_transcript_mean_confidence = mean_confidence;
				state->has_new_transcript.set();
				lock.unlock();
			}

			state->is_bgthread_active.clear();
		}
	}

	// ---------------------------------------------------------------
	// Workload
	// ---------------------------------------------------------------
	struct SpeechToTextWorkload
	{
		SpeechToTextConfig config;
		SpeechToTextInputs inputs;
		SpeechToTextOutputs outputs;
		StatePtr<SpeechToTextState> state;

		void load()
		{
			SpeechToText::initialize(config.settings, state->internal_state);
			state->thread_should_exit = false;
			state->thread_has_work = false;
			state->is_bgthread_active.clear();
			state->has_new_transcript.clear();
			state->is_buffer_swapped.set(false);

			state->bg_thread = Thread(speech_to_text_thread, static_cast<void*>(&state.get()), "SpeechToTextThread");
		}

		void tick(const TickInfo& tick_info)
		{
			// Downsample and append new samples
			{
				AudioBuffer512 downsampled;
				downsample_to_accumulator_rate(inputs.mono.samples, inputs.mono.sample_rate, downsampled);

				AudioAccumulator& foreground_accumulator = state->get_foreground_accumulator();
				foreground_accumulator.end_time_sec = tick_info.time_now;

				// if we don't have room for the full new set of samples, drop the oldest 2 seconds from the accumulator
				if (foreground_accumulator.samples.size() + downsampled.size() > foreground_accumulator.samples.capacity())
				{
					foreground_accumulator.request_drop_oldest_duration_sec(2.0f);
				}

				const size_t old_size = foreground_accumulator.samples.size();
				const size_t add_count = downsampled.size();

				ROBOTICK_ASSERT(old_size + add_count <= foreground_accumulator.samples.capacity());

				memcpy(foreground_accumulator.samples.data() + old_size, downsampled.data(), add_count * sizeof(float));

				foreground_accumulator.samples.set_size(old_size + add_count);

				ROBOTICK_ASSERT(foreground_accumulator.samples.size() <= foreground_accumulator.samples.capacity());
			}

			// Swap buffers if background thread is idle, and we're comfortably beyond the end of a "voiced" piece of audio
			{
				const bool is_voiced = inputs.is_voiced;
				const double now = tick_info.time_now;

				AudioAccumulator& foreground_accumulator = state->get_foreground_accumulator();

				if (is_voiced)
				{
					state->last_voiced_time = now;
					state->is_voiced_segment_pending = foreground_accumulator.get_duration_sec() >= config.settings.min_voiced_duration_sec;
				}

				const double silence_duration = now - state->last_voiced_time;
				const bool should_submit =
					(!is_voiced && state->is_voiced_segment_pending && silence_duration > config.settings.silence_hangover_sec);

				if (should_submit && !state->is_bgthread_active.is_set())
				{
					state->is_voiced_segment_pending = false;
					outputs.transcribe_session_count++;

					state->transcribe_start_time_sec = tick_info.time_now;

					LockGuard lock(state->mutex);

					AudioAccumulator& background_accumulator = state->get_background_accumulator();

					// copy FG → BG
					background_accumulator = foreground_accumulator;

					// swap and clear FG
					state->is_buffer_swapped.set(!state->is_buffer_swapped.is_set());
					state->get_foreground_accumulator().samples.clear();
					state->get_foreground_accumulator().end_time_sec = now;

					// signal thread
					state->thread_has_work = true;
					state->cv.notify_one();
				}
			}

			// Retrieve transcript if ready
			if (state->has_new_transcript.is_set())
			{
				state->has_new_transcript.clear();

				LockGuard lock(state->mutex);
				outputs.words = state->last_result;
				outputs.transcript = state->last_transcript;
				outputs.transcribe_duration_sec = tick_info.time_now - state->transcribe_start_time_sec;
				outputs.transcript_mean_confidence = state->last_transcript_mean_confidence;
			}

			outputs.accumulator_duration_sec = state->get_foreground_accumulator().get_duration_sec();
			outputs.accumulator_capacity_sec = state->get_foreground_accumulator().get_capacity_sec();
			outputs.is_transcribe_thread_active = state->is_bgthread_active.is_set();
		}

		void stop()
		{
			{
				LockGuard lock(state->mutex);
				state->thread_should_exit = true;
				state->cv.notify_one();
			}

			if (state->bg_thread.is_joining_supported() && state->bg_thread.is_joinable())
			{
				state->bg_thread.join();
			}

			SpeechToText::uninitialize(state->internal_state);
		}
	};

} // namespace robotick

#endif // ROBOTICK_PLATFORM_DESKTOP || ROBOTICK_PLATFORM_LINUX
