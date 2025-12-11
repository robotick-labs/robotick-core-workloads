// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#include "robotick/systems/auditory/SpeechToText.h"
#include "robotick/framework/concurrency/Thread.h"
#include "robotick/framework/containers/HeapVector.h"
#include "robotick/systems/audio/WavFile.h"

#include "whisper.h"

#include <catch2/catch_all.hpp>
#include <cmath>
#include <cstdio>

namespace robotick::test
{

	namespace utils
	{
		// Tiny WAV reader (16-bit PCM mono @ 16 kHz).
		// - parses RIFF header and finds the "data" chunk.
		static bool load_wav_s16_mono_16k(const char* path, HeapVector<float>& out)
		{
			FILE* f = ::fopen(path, "rb");
			if (!f)
			{
				return false;
			}

			auto rd32 = [&](uint32_t& v)
			{
				return ::fread(&v, 1, 4, f) == 4;
			};
			auto rd16 = [&](uint16_t& v)
			{
				return ::fread(&v, 1, 2, f) == 2;
			};

			uint32_t riff = 0, riff_size = 0, wave = 0;
			if (!rd32(riff) || !rd32(riff_size) || !rd32(wave))
			{
				::fclose(f);
				return false;
			}
			if (riff != 0x46464952 /*"RIFF"*/ || wave != 0x45564157 /*"WAVE"*/)
			{
				::fclose(f);
				return false;
			}

			uint16_t audio_format = 0, num_channels = 0, bits_per_sample = 0;
			uint32_t sample_rate = 0;
			uint32_t data_size = 0;
			long data_pos = -1;

			while (data_pos < 0 && !::feof(f))
			{
				uint32_t chunk_id = 0, chunk_sz = 0;
				if (!rd32(chunk_id) || !rd32(chunk_sz))
				{
					::fclose(f);
					return false;
				}

				if (chunk_id == 0x20746d66 /*"fmt "*/)
				{
					if (chunk_sz < 16)
					{
						::fclose(f);
						return false;
					}
					if (!rd16(audio_format) || !rd16(num_channels) || !rd32(sample_rate))
					{
						::fclose(f);
						return false;
					}
					uint32_t byte_rate = 0;
					uint16_t block_align = 0;
					if (!rd32(byte_rate) || !rd16(block_align) || !rd16(bits_per_sample))
					{
						::fclose(f);
						return false;
					}
					if (chunk_sz > 16 && ::fseek(f, static_cast<long>(chunk_sz - 16), SEEK_CUR) != 0)
					{
						::fclose(f);
						return false;
					}
				}
				else if (chunk_id == 0x61746164 /*"data"*/)
				{
					data_pos = ::ftell(f);
					data_size = chunk_sz;
					if (::fseek(f, static_cast<long>(chunk_sz), SEEK_CUR) != 0)
					{
						::fclose(f);
						return false;
					}
				}
				else
				{
					if (::fseek(f, static_cast<long>(chunk_sz), SEEK_CUR) != 0)
					{
						::fclose(f);
						return false;
					}
				}
			}

			if (data_pos < 0 || audio_format != 1 || num_channels != 1 || bits_per_sample != 16 || sample_rate != 16000)
			{
				::fclose(f);
				return false;
			}

			if (::fseek(f, data_pos, SEEK_SET) != 0)
			{
				::fclose(f);
				return false;
			}

			const size_t num_samples = data_size / 2;
			HeapVector<int16_t> pcm16;
			pcm16.initialize(num_samples);
			if (::fread(pcm16.data(), sizeof(int16_t), num_samples, f) != num_samples)
			{
				::fclose(f);
				return false;
			}
			::fclose(f);

			out.initialize(num_samples);
			for (size_t i = 0; i < num_samples; ++i)
			{
				out[i] = static_cast<float>(pcm16[i]) / 32768.0f;
			}
			return true;
		}

	} // namespace utils

	TEST_CASE("Unit/Systems/Auditory/SpeechToText")
	{
		const char* model_path = "data/models/whisper/ggml-base.en.bin";
		const char* wav_path_jfk = "data/wav/jfk.wav";

		// Expected JFK transcription as word-level spans
		static const TranscribedWord expected_words_jfk[] = {
			{"[_BEG_]", 0.000000f, 0.000000f},
			{" And", 0.320000f, 0.370000f},
			{" so", 0.370000f, 0.530000f},
			{" my", 0.690000f, 0.850000f},
			{" fellow", 0.850000f, 1.590000f},
			{" Americans", 1.590000f, 2.100000f},
			{",", 2.850000f, 3.300000f},
			{" ask", 3.300000f, 4.140000f},
			{" not", 4.140000f, 4.280000f},
			{" what", 5.030000f, 5.350000f},
			{" your", 5.410000f, 5.740000f},
			{" country", 5.740000f, 6.410000f},
			{" can", 6.410000f, 6.740000f},
			{" do", 6.740000f, 6.920000f},
			{" for", 7.000000f, 7.000000f},
			{" you", 7.010000f, 7.520000f},
			{",", 7.810000f, 8.050000f},
			{" ask", 8.190000f, 8.370000f},
			{" what", 8.370000f, 8.750000f},
			{" you", 8.910000f, 9.040000f},
			{" can", 9.040000f, 9.320000f},
			{" do", 9.320000f, 9.380000f},
			{" for", 9.440000f, 9.760000f},
			{" your", 9.760000f, 9.990000f},
			{" country", 10.020000f, 10.360000f},
			{".", 10.510000f, 10.990000f},
			{"[_TT_550]", 11.000000f, 11.000000f},
		};

		static constexpr size_t num_expected_words_jfk = sizeof(expected_words_jfk) / sizeof(expected_words_jfk[0]);

		SECTION("Transcript clear resets all fields")
		{
			Transcript transcript;
			TranscribedWord word;
			word.text = "Hello";
			word.start_time_sec = 1.0f;
			word.end_time_sec = 1.5f;
			word.confidence = 0.42f;

			transcript.words.add(word);
			transcript.text = "Hello";
			transcript.transcribe_duration_sec = 3.0f;
			transcript.transcript_mean_confidence = 0.31f;
			transcript.start_time_sec = 5.0f;
			transcript.duration_sec = 2.0f;

			transcript.clear();

			CHECK(transcript.words.empty());
			CHECK(transcript.text.empty());
				CHECK(transcript.transcribe_duration_sec == Catch::Approx(0.0f));
				CHECK(transcript.transcript_mean_confidence == Catch::Approx(0.0f));
				CHECK(transcript.start_time_sec == Catch::Approx(0.0f));
				CHECK(transcript.duration_sec == Catch::Approx(0.0f));
		}

		SECTION("Transcript timing prefers word spans but falls back to defaults")
		{
			Transcript transcript;

			transcript.update_timing_from_words(1.25f, 2.5f);
				CHECK(transcript.start_time_sec == Catch::Approx(1.25f));
				CHECK(transcript.duration_sec == Catch::Approx(2.5f));

			TranscribedWord first_word;
			first_word.text = "Hi";
			first_word.start_time_sec = 0.5f;
			first_word.end_time_sec = 1.0f;

			TranscribedWord last_word;
			last_word.text = "all";
			last_word.start_time_sec = 2.0f;
			last_word.end_time_sec = 2.4f;

			transcript.words.add(first_word);
			transcript.words.add(last_word);
			transcript.update_timing_from_words(0.0f, 0.0f);

				CHECK(transcript.start_time_sec == Catch::Approx(first_word.start_time_sec));
				CHECK(transcript.duration_sec == Catch::Approx(last_word.end_time_sec - first_word.start_time_sec));
		}

		SECTION("SpeechToText transcribes JFK WAV correctly")
		{
			FILE* model_file = ::fopen(model_path, "rb");
			if (!model_file)
			{
				WARN("Skipping SpeechToText transcription test because Whisper model is not available");
				return;
			}
			::fclose(model_file);

			// load our wav-file in required format
			HeapVector<float> pcmf32;
			REQUIRE(utils::load_wav_s16_mono_16k(wav_path_jfk, pcmf32));
			REQUIRE(!pcmf32.empty());
			REQUIRE(pcmf32.size() == 176000);

			// set up SpeechToText - loading model etc
			SpeechToTextSettings settings;
			settings.model_path = model_path;
			settings.num_threads = Thread::get_hardware_concurrency(); // allow use of all available threads to keep test fast

			SpeechToTextInternalState state;

			SpeechToText::initialize(settings, state);

			// perform our transcription
			TranscribedWords words;
			const bool success = SpeechToText::transcribe(state, pcmf32.data(), pcmf32.size(), words);

			// ensure the results are what we expect:
			REQUIRE(success);

			REQUIRE(num_expected_words_jfk == 27);
			REQUIRE(words.size() == num_expected_words_jfk);

			for (size_t word_index = 0; word_index < words.size(); word_index++)
			{
				const TranscribedWord& word = words[word_index];
				const TranscribedWord& expected_word = expected_words_jfk[word_index];

				CHECK(word.text == expected_word.text);
				CHECK(word.start_time_sec == Catch::Approx(expected_word.start_time_sec).margin(0.01f));
				CHECK(word.end_time_sec == Catch::Approx(expected_word.end_time_sec).margin(0.01f));
			}
		}
	}

} // namespace robotick::test
