// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#include "robotick/systems/audio/WavFile.h"

#include <catch2/catch_all.hpp>

namespace robotick::test
{

	TEST_CASE("Unit/Systems/Audio/WavFile")
	{
		SECTION("Loads valid mono WAV")
		{
			WavFile wav;
			bool result = wav.load("data/wav/mono_valid.wav");
			REQUIRE(result);
			REQUIRE(wav.get_frame_count() == 44100);
			REQUIRE(wav.get_sample_rate() == 44100);
			REQUIRE(wav.get_num_channels() == 1);
			REQUIRE(wav.get_duration_seconds() == 1.0f);
		}

		SECTION("Loads valid stereo WAV")
		{
			WavFile wav;
			bool result = wav.load("data/wav/stereo_valid.wav");
			REQUIRE(result);
			REQUIRE(wav.get_frame_count() == 44100);
			REQUIRE(wav.get_sample_rate() == 44100);
			REQUIRE(wav.get_num_channels() == 2);
			REQUIRE(wav.get_duration_seconds() == 1.0f);
		}

		SECTION("Rejects truncated WAV")
		{
			WavFile wav;
			bool result = wav.load("data/wav/truncated.wav");
			REQUIRE_FALSE(result);
		}

		SECTION("Rejects invalid RIFF header")
		{
			WavFile wav;
			bool result = wav.load("data/wav/bad_header.wav");
			REQUIRE_FALSE(result);
		}
	}

} // namespace robotick::test
