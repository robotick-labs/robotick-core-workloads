// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#include "robotick/systems/auditory/SpeechToText.h"
#include "robotick/systems/audio/WavFile.h"

#include <catch2/catch_all.hpp>
#include <cmath>
#include <filesystem>

namespace robotick::test
{

	TEST_CASE("Unit/Systems/Auditory/SpeechToText")
	{
		ROBOTICK_INFO("Current working dir: '%s'", std::filesystem::current_path().string().c_str());
		SpeechToText::reset();
		SpeechToText::init();

		SECTION("Transcript is empty on init")
		{
			auto& t = SpeechToText::transcript();
			REQUIRE(t.size() == 0);
		}

		SECTION("Push mono WAV into ring and extract window")
		{
			WavFile wav;
			REQUIRE(wav.load("data/wav/mono_valid.wav"));

			REQUIRE(wav.get_sample_rate() == 44100);
			REQUIRE(wav.get_num_channels() == 1);
			REQUIRE(wav.get_frame_count() == 44100);
			REQUIRE(wav.get_duration_seconds() == 1.0f);

			const float* mono = wav.get_left_samples().data();
			REQUIRE(mono != nullptr);

			const float start_time = 123.0f;
			SpeechToText::push_audio(mono, wav.get_frame_count(), wav.get_sample_rate(), start_time);

			SpeechToTextBuffer query;
			query.start_time = 123.1f;
			query.end_time = 123.3f;

			const float* view = nullptr;
			size_t count = 0;
			size_t rate = 0;

			REQUIRE(SpeechToText::get_audio_window(query, view, count, rate));
			REQUIRE(view != nullptr);
			REQUIRE(count > 0);
			REQUIRE(rate == speech_to_text::target_sample_rate_hz);
		}

		SECTION("Rejects query with no audio pushed")
		{
			SpeechToText::reset();
			SpeechToText::init();

			SpeechToTextBuffer query{100.0f, 101.0f};
			const float* view = nullptr;
			size_t count = 0, rate = 0;
			REQUIRE_FALSE(SpeechToText::get_audio_window(query, view, count, rate));
		}

		SECTION("Handles resample from 48000Hz")
		{
			constexpr size_t frames = 48000;
			std::vector<float> samples(frames);
			for (size_t i = 0; i < frames; ++i)
				samples[i] = std::sin(2 * 3.14159f * 440.0f * i / 48000.0f);

			SpeechToText::push_audio(samples.data(), frames, 48000, 200.0f);

			SpeechToTextBuffer query{200.2f, 200.6f};
			const float* view = nullptr;
			size_t count = 0, rate = 0;
			REQUIRE(SpeechToText::get_audio_window(query, view, count, rate));
			REQUIRE(rate == speech_to_text::target_sample_rate_hz);
			REQUIRE(count > 0);
		}

		SECTION("Multiple push_audio calls form continuous stream")
		{
			std::vector<float> block(22050, 0.5f); // 0.5s @ 44100Hz
			SpeechToText::push_audio(block.data(), block.size(), 44100, 300.0f);
			SpeechToText::push_audio(block.data(), block.size(), 44100, 300.5f);

			SpeechToTextBuffer query{300.3f, 300.7f};
			const float* view = nullptr;
			size_t count = 0, rate = 0;
			REQUIRE(SpeechToText::get_audio_window(query, view, count, rate));
			REQUIRE(count > 0);
		}

		SECTION("Ring buffer wraparound does not crash")
		{
			std::vector<float> block(16000, 0.25f); // 1s @ 16kHz
			const float base_time = 500.0f;
			for (int i = 0; i < 12; ++i)
			{
				float t = base_time + static_cast<float>(i);
				SpeechToText::push_audio(block.data(), block.size(), 16000, t);
			}

			SpeechToTextBuffer query{base_time + 10.0f, base_time + 10.5f};
			const float* view = nullptr;
			size_t count = 0, rate = 0;
			REQUIRE(SpeechToText::get_audio_window(query, view, count, rate));
			REQUIRE(count > 0);
			REQUIRE(rate == speech_to_text::target_sample_rate_hz);
		}

		SECTION("Push WAV and simulate transcript")
		{
			WavFile wav;
			REQUIRE(wav.load("data/wav/mono_valid.wav"));
			const float* mono = wav.get_left_samples().data();
			REQUIRE(mono);
			REQUIRE(wav.get_frame_count() > 0);

			// Push whole file at time 100.0
			SpeechToText::push_audio(mono, wav.get_frame_count(), wav.get_sample_rate(), 100.0f);

			// Query entire span
			SpeechToTextBuffer buffer;
			buffer.start_time = 100.0f;
			buffer.end_time = 101.0f;

			const float* view = nullptr;
			size_t count = 0;
			size_t rate = 0;
			REQUIRE(SpeechToText::get_audio_window(buffer, view, count, rate));
			REQUIRE(view);
			REQUIRE(count > 0);
			REQUIRE(rate == speech_to_text::target_sample_rate_hz);

			const auto& transcript = SpeechToText::transcript();

			REQUIRE(transcript.size() == 2);
			REQUIRE(transcript[0].word == "hello");
			REQUIRE(transcript[1].word == "robotick");
			REQUIRE(transcript[0].engine_time >= buffer.start_time);
			REQUIRE(transcript[1].engine_time <= buffer.end_time);
		}
	}

} // namespace robotick::test