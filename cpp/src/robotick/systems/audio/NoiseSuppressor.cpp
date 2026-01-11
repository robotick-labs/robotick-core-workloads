// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#include "robotick/systems/audio/NoiseSuppressor.h"

#include "robotick/framework/math/Clamp.h"
#include "robotick/framework/math/MathUtils.h"

#include <cmath>

namespace robotick
{
	ROBOTICK_REGISTER_STRUCT_BEGIN(NoiseSuppressorConfig)
	ROBOTICK_STRUCT_FIELD(NoiseSuppressorConfig, float, noise_learning_rms_threshold)
	ROBOTICK_STRUCT_FIELD(NoiseSuppressorConfig, float, noise_profile_alpha)
	ROBOTICK_STRUCT_FIELD(NoiseSuppressorConfig, float, suppression_strength)
	ROBOTICK_STRUCT_FIELD(NoiseSuppressorConfig, float, min_gain)
	ROBOTICK_STRUCT_FIELD(NoiseSuppressorConfig, float, gain_smooth_alpha)
	ROBOTICK_STRUCT_FIELD(NoiseSuppressorConfig, float, noise_only_rms_threshold)
	ROBOTICK_STRUCT_FIELD(NoiseSuppressorConfig, float, noise_floor_min)
	ROBOTICK_REGISTER_STRUCT_END(NoiseSuppressorConfig)

	void NoiseSuppressor::plan_fft(NoiseSuppressorState& state)
	{
		// Pre-size buffers and allocate fixed-heap FFT plans (fallback to heap if needed).
		state.time_domain.set_size(NoiseSuppressorState::frame_size);
		state.ifft_time_domain.set_size(NoiseSuppressorState::frame_size);
		state.fft_output.set_size(NoiseSuppressorState::fft_bins);
		state.fft_processed.set_size(NoiseSuppressorState::fft_bins);
		state.noise_floor.set_size(NoiseSuppressorState::fft_bins);
		state.gain_smooth.set_size(NoiseSuppressorState::fft_bins);

		size_t kiss_cfg_length_bytes = sizeof(state.kiss_cfg_mem_fwd);
		state.kiss_config_fwd = kiss_fftr_alloc(static_cast<int>(NoiseSuppressorState::fft_size), 0, state.kiss_cfg_mem_fwd, &kiss_cfg_length_bytes);
		if (!state.kiss_config_fwd)
		{
			state.kiss_config_fwd = kiss_fftr_alloc(static_cast<int>(NoiseSuppressorState::fft_size), 0, nullptr, nullptr);
		}
		ROBOTICK_ASSERT(state.kiss_config_fwd && "kiss_fftr_alloc failed for forward FFT");

		kiss_cfg_length_bytes = sizeof(state.kiss_cfg_mem_inv);
		state.kiss_config_inv = kiss_fftr_alloc(static_cast<int>(NoiseSuppressorState::fft_size), 1, state.kiss_cfg_mem_inv, &kiss_cfg_length_bytes);
		if (!state.kiss_config_inv)
		{
			state.kiss_config_inv = kiss_fftr_alloc(static_cast<int>(NoiseSuppressorState::fft_size), 1, nullptr, nullptr);
		}
		ROBOTICK_ASSERT(state.kiss_config_inv && "kiss_fftr_alloc failed for inverse FFT");
	}

	void NoiseSuppressor::build_window(NoiseSuppressorState& state)
	{
		// Hann window with RMS normalization so inverse FFT amplitudes remain stable.
		state.window.set_size(NoiseSuppressorState::frame_size);
		double energy = 0.0;
		for (size_t i = 0; i < NoiseSuppressorState::frame_size; ++i)
		{
			const float phase = static_cast<float>(2.0 * M_PI * static_cast<double>(i) / (NoiseSuppressorState::frame_size - 1));
			const float w = 0.5f * (1.0f - cosf(phase));
			state.window[i] = w;
			energy += static_cast<double>(w) * static_cast<double>(w);
		}

		const double mean_energy = energy / static_cast<double>(NoiseSuppressorState::frame_size);
		state.window_rms = static_cast<float>(sqrt(mean_energy));
		if (state.window_rms <= 1e-6f)
		{
			state.window_rms = 1.0f;
		}
	}

	void NoiseSuppressor::reset_state(NoiseSuppressorState& state)
	{
		// Initialize noise profile and smoothing gains to neutral defaults.
		for (size_t i = 0; i < NoiseSuppressorState::fft_bins; ++i)
		{
			state.noise_floor[i] = 1e-6f;
			state.gain_smooth[i] = 1.0f;
			state.fft_output[i].r = 0.0f;
			state.fft_output[i].i = 0.0f;
			state.fft_processed[i].r = 0.0f;
			state.fft_processed[i].i = 0.0f;
		}

		for (size_t i = 0; i < NoiseSuppressorState::frame_size; ++i)
		{
			state.time_domain[i] = 0.0f;
			state.ifft_time_domain[i] = 0.0f;
		}
	}

	void NoiseSuppressor::process_frame(const NoiseSuppressorConfig& config,
		NoiseSuppressorState& state,
		const AudioFrame& input,
		AudioFrame& output,
		bool& is_noise_only,
		NoiseSuppressorOutputs& debug_outputs)
	{
		// Flow: window → FFT → update noise floor → compute gains → IFFT → normalize.
		if (!state.kiss_config_fwd || !state.kiss_config_inv)
		{
			plan_fft(state);
		}
		if (state.window.empty())
		{
			build_window(state);
		}

		const size_t input_samples = input.samples.size();
		output.sample_rate = input.sample_rate;
		output.timestamp = input.timestamp;
		output.samples.clear();

		if (input_samples == 0)
		{
			is_noise_only = true;
			debug_outputs.noise_floor_rms = 0.0f;
			return;
		}

		double energy = 0.0;
		for (size_t i = 0; i < input_samples; ++i)
		{
			const float sample = input.samples[i];
			energy += static_cast<double>(sample) * static_cast<double>(sample);
		}
		const float rms = static_cast<float>(sqrt(energy / static_cast<double>(input_samples)));

		const bool learn_noise = rms <= config.noise_learning_rms_threshold;
		is_noise_only = rms <= config.noise_only_rms_threshold;

		for (size_t i = 0; i < NoiseSuppressorState::frame_size; ++i)
		{
			const float sample = (i < input_samples) ? input.samples[i] : 0.0f;
			state.time_domain[i] = sample * state.window[i];
		}

		kiss_fftr(state.kiss_config_fwd, state.time_domain.data(), state.fft_output.data());

		// Clamp tunables to safe ranges.
		const float alpha = robotick::clamp(config.noise_profile_alpha, 0.0f, 1.0f);
		const float smooth_alpha = robotick::clamp(config.gain_smooth_alpha, 0.0f, 1.0f);
		const float suppression_strength = robotick::clamp(config.suppression_strength, 0.0f, 1.0f);
		const float min_gain = robotick::clamp(config.min_gain, 0.0f, 1.0f);
		const float floor_min = robotick::max(config.noise_floor_min, 1e-12f);

		// Per-bin suppression using a simple Wiener-style gain.
		double noise_floor_energy = 0.0;
		for (size_t bin = 0; bin < NoiseSuppressorState::fft_bins; ++bin)
		{
			const float real_part = state.fft_output[bin].r;
			const float imag_part = state.fft_output[bin].i;
			const float magnitude = sqrtf(real_part * real_part + imag_part * imag_part) + 1e-12f;

			float noise_floor = state.noise_floor[bin];
			if (learn_noise)
			{
				noise_floor = (1.0f - alpha) * noise_floor + alpha * magnitude;
			}
			noise_floor = robotick::max(noise_floor, floor_min);
			state.noise_floor[bin] = noise_floor;
			noise_floor_energy += static_cast<double>(noise_floor) * static_cast<double>(noise_floor);

			const float noise_ratio = noise_floor / magnitude;
			float gain = 1.0f - suppression_strength * noise_ratio;
			gain = robotick::clamp(gain, min_gain, 1.0f);

			float smoothed_gain = state.gain_smooth[bin];
			smoothed_gain = (1.0f - smooth_alpha) * smoothed_gain + smooth_alpha * gain;
			state.gain_smooth[bin] = smoothed_gain;

			state.fft_processed[bin].r = real_part * smoothed_gain;
			state.fft_processed[bin].i = imag_part * smoothed_gain;
		}

		const double mean_noise_energy = noise_floor_energy / static_cast<double>(NoiseSuppressorState::fft_bins);
		debug_outputs.noise_floor_rms = static_cast<float>(sqrt(mean_noise_energy));

		kiss_fftri(state.kiss_config_inv, state.fft_processed.data(), state.ifft_time_domain.data());

		const float normalizer = 1.0f / (static_cast<float>(NoiseSuppressorState::fft_size) * state.window_rms);
		output.samples.set_size(input_samples);
		for (size_t i = 0; i < input_samples; ++i)
		{
			output.samples[i] = state.ifft_time_domain[i] * normalizer;
		}
	}
} // namespace robotick
