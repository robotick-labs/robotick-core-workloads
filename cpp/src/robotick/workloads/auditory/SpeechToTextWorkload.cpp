// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#include "robotick/api.h"
#include "robotick/platform/Threading.h"
#include "robotick/systems/audio/AudioFrame.h"
#include "robotick/systems/auditory/SpeechToText.h"

#include <atomic>
#include <cmath>
#include <condition_variable>
#include <cstring> // for std::memmove
#include <mutex>

namespace robotick
{
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

		AudioAccumulator audio_accumulators[2];
		std::atomic<bool> is_buffer_swapped{false}; // false → [0] is FG, true → [1] is FG

		TranscribedWords last_result;
		FixedString512 last_transcript;

		AtomicFlag is_bgthread_active;
		AtomicFlag has_new_transcript;

		Thread bg_thread;
		std::mutex mutex;
		std::condition_variable cv;
		bool thread_should_exit = false;
		bool thread_has_work = false;

		AudioAccumulator& get_fg() { return is_buffer_swapped.load() ? audio_accumulators[1] : audio_accumulators[0]; }

		AudioAccumulator& get_bg() { return is_buffer_swapped.load() ? audio_accumulators[0] : audio_accumulators[1]; }
	};

	// ---------------------------------------------------------------
	// Simple linear downsampler to 16 kHz
	// ---------------------------------------------------------------
	static void downsample_to_16k(const AudioBuffer512& input, const uint32_t input_rate, AudioBuffer512& output)
	{
		const uint32_t output_rate = 16000.0f;
		const float ratio = (float)input_rate / (float)output_rate;
		const size_t dst_count = static_cast<size_t>((float)input.size() / ratio);

		for (size_t dst_index = 0; dst_index < dst_count; ++dst_index)
		{
			const float src_index_f = dst_index * ratio;
			const size_t src_index_i = static_cast<size_t>(src_index_f);
			const float frac = src_index_f - static_cast<float>(src_index_i);

			const float sample =
				(src_index_i + 1 < input.size()) ? input[src_index_i] * (1.0f - frac) + input[src_index_i + 1] * frac : input[src_index_i];

			if (output.size() < output.capacity())
			{
				output.add(sample);
			}
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

			const AudioAccumulator& audio_accumulator = state->get_bg();

			if (!audio_accumulator.empty())
			{
				TranscribedWords result;
				FixedString512 transcript;

				SpeechToText::transcribe(state->internal_state, audio_accumulator.data(), audio_accumulator.size(), result);

				lock.lock();
				state->last_result = result;
				state->last_transcript = transcript;
				state->has_new_transcript.set();
				lock.unlock();
			}

			state->is_bgthread_active.unset();
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
			state->is_bgthread_active.unset();
			state->has_new_transcript.unset();
			state->is_buffer_swapped.store(false);

			state->bg_thread = Thread(speech_to_text_thread, static_cast<void*>(&state.get()), "SpeechToTextThread");
		}

		void tick(const TickInfo& tick_info)
		{
			(void)tick_info;

			AudioBuffer512 downsampled;
			downsample_to_16k(inputs.mono.samples, inputs.mono.sample_rate, downsampled);

			AudioAccumulator& fg = state->get_fg();

			// Append new samples
			for (size_t i = 0; i < downsampled.size(); ++i)
			{
				if (fg.size() >= fg.capacity())
				{
					// Sliding window: keep last 9 s, drop oldest 1 s
					const size_t keep_count = 9 * 16000;
					const size_t drop_count = fg.size() - keep_count;
					std::memmove(fg.data(), fg.data() + drop_count, keep_count * sizeof(float));

					fg.set_size(keep_count);
				}
				fg.add(downsampled[i]);
			}

			// Swap buffers if background thread is idle
			if (!state->is_bgthread_active.is_set())
			{
				std::lock_guard<std::mutex> lock(state->mutex);

				AudioAccumulator& bg = state->get_bg();
				bg = fg;	// copy current FG into BG
				fg.clear(); // start fresh

				// Toggle active buffer
				state->is_buffer_swapped.store(!state->is_buffer_swapped.load());

				// Signal background thread to process
				state->thread_has_work = true;
				state->cv.notify_one();
			}

			// Retrieve transcript if ready
			if (state->has_new_transcript.is_set())
			{
				state->has_new_transcript.unset();
				outputs.words = state->last_result;
				outputs.transcript = state->last_transcript;
			}
		}

		void shutdown()
		{
			{
				std::lock_guard<std::mutex> lock(state->mutex);
				state->thread_should_exit = true;
				state->cv.notify_one();
			}

			if (state->bg_thread.is_joining_supported() && state->bg_thread.is_joinable())
			{
				state->bg_thread.join();
			}

			// No SpeechToText::shutdown() currently defined; omit safely
		}
	};

} // namespace robotick
