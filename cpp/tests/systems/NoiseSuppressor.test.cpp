// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#include "robotick/systems/audio/NoiseSuppressor.h"
#include "robotick/systems/audio/AudioFrame.h"

#include <catch2/catch_all.hpp>
#include <cmath>

namespace robotick::test
{
	namespace
	{
		float compute_rms(const AudioFrame& frame)
		{
			if (frame.samples.empty())
			{
				return 0.0f;
			}

			double energy = 0.0;
			for (float sample : frame.samples)
			{
				energy += static_cast<double>(sample) * static_cast<double>(sample);
			}
			return static_cast<float>(sqrt(energy / static_cast<double>(frame.samples.size())));
		}

		void fill_white_noise(AudioFrame& frame, float amplitude, uint32_t& seed)
		{
			frame.samples.set_size(frame.samples.capacity());
			for (size_t i = 0; i < frame.samples.size(); ++i)
			{
				seed = 1664525u * seed + 1013904223u;
				const float rand01 = static_cast<float>((seed >> 8) & 0x00FFFFFF) / static_cast<float>(0x01000000);
				const float sample = (rand01 * 2.0f - 1.0f) * amplitude;
				frame.samples[i] = sample;
			}
		}

		void fill_sine(AudioFrame& frame, float amplitude, float frequency_hz)
		{
			const float sample_rate = static_cast<float>(frame.sample_rate);
			const float phase_step = 2.0f * static_cast<float>(M_PI) * frequency_hz / sample_rate;
			frame.samples.set_size(frame.samples.capacity());
			float phase = 0.0f;
			for (size_t i = 0; i < frame.samples.size(); ++i)
			{
				frame.samples[i] = amplitude * sinf(phase);
				phase += phase_step;
			}
		}
	} // namespace

	TEST_CASE("Unit/Systems/NoiseSuppressor/LearnsNoiseProfileAndSuppresses")
	{
		NoiseSuppressorConfig config{};
		config.noise_learning_rms_threshold = 1.0f;
		config.noise_profile_alpha = 0.5f;
		config.suppression_strength = 1.0f;
		config.min_gain = 0.1f;
		config.gain_smooth_alpha = 1.0f;
		config.noise_only_rms_threshold = 0.2f;

		NoiseSuppressorState state{};
		NoiseSuppressorOutputs debug{};
		NoiseSuppressor::plan_fft(state);
		NoiseSuppressor::build_window(state);
		NoiseSuppressor::reset_state(state);

		AudioFrame input{};
		input.sample_rate = 16000;
		uint32_t seed = 7u;
		fill_white_noise(input, 0.05f, seed);

		AudioFrame output{};
		bool is_noise_only = false;
		NoiseSuppressor::process_frame(config, state, input, output, is_noise_only, debug);

		const float input_rms = compute_rms(input);
		const float output_rms = compute_rms(output);

		CHECK(debug.noise_floor_rms > 0.0f);
		CHECK(output_rms < input_rms);
		CHECK(is_noise_only);
	}

	TEST_CASE("Unit/Systems/NoiseSuppressor/PreservesStrongSignal")
	{
		NoiseSuppressorConfig config{};
		config.noise_learning_rms_threshold = 0.01f;
		config.noise_profile_alpha = 0.2f;
		config.suppression_strength = 0.6f;
		config.min_gain = 0.2f;
		config.gain_smooth_alpha = 1.0f;
		config.noise_only_rms_threshold = 0.05f;

		NoiseSuppressorState state{};
		NoiseSuppressorOutputs debug{};
		NoiseSuppressor::plan_fft(state);
		NoiseSuppressor::build_window(state);
		NoiseSuppressor::reset_state(state);

		AudioFrame input{};
		input.sample_rate = 16000;
		fill_sine(input, 0.2f, 440.0f);

		AudioFrame output{};
		bool is_noise_only = false;
		NoiseSuppressor::process_frame(config, state, input, output, is_noise_only, debug);

		const float input_rms = compute_rms(input);
		const float output_rms = compute_rms(output);

		CHECK(output_rms > input_rms * 0.2f);
		CHECK_FALSE(is_noise_only);
	}
} // namespace robotick::test
