// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#include "robotick/api.h"
#include "robotick/platform/Threading.h"
#include "robotick/systems/audio/AudioFrame.h"
#include "robotick/systems/auditory/SpeechToText.h"

#include <cmath>
#include <condition_variable>
#include <mutex>
#include <thread>

namespace robotick
{
#if 0
	struct SpeechToTextInputs
	{
		AudioFrame mono;
	};

	struct SpeechToTextOutputs
	{
		TranscribedWords words;
		FixedString512 transcript;
	};

	static constexpr uint32_t accumulator_capacity_sec = 10;
	static constexpr uint32_t accumulator_sample_rate_hz = 16000;

	using AudioAccumulator = FixedVector<float, accumulator_capacity_sec * accumulator_sample_rate_hz>;

	struct SpeechToTextState
	{
		SpeechToTextInternalState internal_state;

		// Double-buffered accumulators (10 s @ 16 kHz)
		AudioAccumulator audio_accumulator[2];
		uint8_t fg_buffer_id = 0;
		uint8_t bg_buffer_id = 1;

		AtomicFlag is_bgthread_active;
		AtomicFlag has_new_transcript;

		std::thread bg_thread;
		std::mutex mutex;
		std::condition_variable cv;
		bool thread_should_exit = false;
		bool thread_has_work = false;
	};

	// ---------------------------------------------------------------
	// Simple linear downsampler to 16 kHz
	// ---------------------------------------------------------------
	static void downsample_to_16k(const float* src, size_t src_count, float src_rate, AudioAccumulator& dst)
	{
		const float dst_rate = 16000.0f;
		const float ratio = src_rate / dst_rate;
		const size_t dst_count = static_cast<size_t>(src_count / ratio);

		for (size_t dst_index = 0; dst_index < dst_count; ++dst_index)
		{
			const float src_index_f = dst_index * ratio;
			const size_t src_index_i = static_cast<size_t>(src_index_f);
			const float frac = src_index_f - static_cast<float>(src_index_i);

			const float sample = (src_index_i + 1 < src_count) ? src[src_index_i] * (1.0f - frac) + src[src_index_i + 1] * frac : src[src_index_i];
			if (dst.size() < dst.capacity())
			{
				dst.add(sample);
			}
		}
	}

	// ---------------------------------------------------------------
	// Background inference thread
	// ---------------------------------------------------------------
	static void speech_to_text_thread(SpeechToTextState* state)
	{
		while (!state->thread_should_exit)
		{
			std::unique_lock<std::mutex> lock(state->mutex);
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

			const uint8_t buffer_id = state->bg_buffer_id;
			const AudioAccumulator& audio_accumulator = state->audio_accumulator[buffer_id];

			if (!audio_accumulator.empty())
			{
				TranscribedWords result;
				FixedString512 transcript;
				SpeechToText::transcribe(state->internal_state, audio_accumulator.data(), audio_accumulator.size(), result);

				lock.lock();
				state->internal_state.last_result = result;
				state->internal_state.last_transcript = transcript;
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
		State<SpeechToTextState> state;

		void load()
		{
			SpeechToText::initialize(config, state->internal_state);
			state->is_bgthread_active.clear();
			state->has_new_transcript.clear();
			state->bg_thread = std::thread(speech_to_text_thread, state.get());
		}

		void tick(const TickInfo& tick_info)
		{
			(void)tick_info;
			const AudioFrame& mono = inputs.mono;

			// downsample new frame to 16 kHz
			FixedVector<float, 4096> downsampled;
			downsample_to_16k(mono.samples.data(), mono.samples.size(), static_cast<float>(mono.sample_rate), downsampled);

			AudioAccumulator& audio_accumulator_fg = state->audio_accumulator[state->fg_buffer_id];

			// append new samples
			for (size_t i = 0; i < downsampled.size(); ++i)
			{
				if (audio_accumulator_fg.size() >= audio_accumulator_fg.capacity())
				{
					// Sliding window: keep last 9 s, drop oldest 1 s
					const size_t keep_count = 9 * 16000;
					const size_t drop_count = audio_accumulator_fg.size() - keep_count;
					std::memmove(fg.data(), fg.data() + drop_count, keep_count * sizeof(float));
					audio_accumulator_fg.resize(keep_count);
				}

				audio_accumulator_fg.add(downsampled[i]);
			}

			// if background thread finished last batch → swap immediately
			if (!state->is_bgthread_active.test())
			{
				std::lock_guard<std::mutex> lock(state->mutex);

				// Copy current fg contents into the soon-to-be background buffer
				state->audio_accumulator[state->bg_buffer_id] = state->audio_accumulator[state->fg_buffer_id];

				// Swap buffer roles
				std::swap(state->fg_buffer_id, state->bg_buffer_id);

				// Clear new fg buffer to start accumulating fresh samples
				state->audio_accumulator[state->fg_buffer_id].clear();

				// Signal background thread to process
				state->thread_has_work = true;
				state->cv.notify_one();
			}

			// retrieve completed transcript if ready
			if (state->has_new_transcript.test())
			{
				state->has_new_transcript.clear();
				outputs.words = state->internal_state.last_result;
				outputs.transcript = state->internal_state.last_transcript;
			}
		}

		void shutdown()
		{
			{
				std::lock_guard<std::mutex> lock(state->mutex);
				state->thread_should_exit = true;
				state->cv.notify_one();
			}
			if (state->bg_thread.joinable())
			{
				state->bg_thread.join();
			}
			SpeechToText::shutdown(state->internal_state);
		}
	};
#endif // #if 0
} // namespace robotick
