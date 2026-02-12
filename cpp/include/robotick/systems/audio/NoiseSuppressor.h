// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0
//
// NoiseSuppressor: FFT-domain stationary-noise suppression.
// Approach: learn a running noise-floor fingerprint (EMA of magnitudes on low-RMS frames),
// then apply smooth, proportional attenuation per bin (Wiener-style) with gain smoothing
// and a minimum gain floor to preserve transients/sibilants.

#pragma once

#include "robotick/api.h"
#include "robotick/framework/containers/FixedVector.h"
#include "robotick/systems/audio/AudioFrame.h"

#include <cstdlib>
#include <kissfft/kiss_fftr.h>

namespace robotick
{
	struct NoiseSuppressorConfig
	{
		float noise_learning_rms_threshold = 0.02f; // RMS threshold for noise-only frames used to learn the profile
		float noise_profile_alpha = 0.1f;			// EMA update rate for the noise fingerprint
		float suppression_strength = 0.8f;			// Overall attenuation strength derived from the profile
		float min_gain = 0.1f;						// Floor on gain to preserve faint transients/sibilants
		float gain_smooth_alpha = 0.2f;				// Smoothing of gain changes to reduce pumping artifacts
		float noise_only_rms_threshold = 0.02f;		// RMS threshold to flag is_noise_only for downstream gating
		float noise_floor_min = 1e-6f;				// Lower bound for noise floor to avoid divide-by-zero/over-attenuation
	};

	struct NoiseSuppressorState
	{
		// FFT geometry based on AudioFrame::AudioBuffer512.
		static constexpr size_t frame_size = AudioBuffer512::capacity();
		static constexpr size_t fft_size = frame_size;
		static constexpr size_t fft_bins = fft_size / 2 + 1;

		// Windowing and time-domain buffers.
		FixedVector<float, frame_size> window;
		FixedVector<float, frame_size> time_domain;
		FixedVector<float, frame_size> ifft_time_domain;
		// Frequency-domain buffers.
		FixedVector<kiss_fft_cpx, fft_bins> fft_output;
		FixedVector<kiss_fft_cpx, fft_bins> fft_processed;
		// Learned noise floor and smoothed gains per bin.
		FixedVector<float, fft_bins> noise_floor;
		FixedVector<float, fft_bins> gain_smooth;

		// Fixed-heap FFT configs (no dynamic allocation after init).
		kiss_fftr_cfg kiss_config_fwd = nullptr;
		kiss_fftr_cfg kiss_config_inv = nullptr;
		alignas(16) unsigned char kiss_cfg_mem_fwd[65536]{};
		alignas(16) unsigned char kiss_cfg_mem_inv[65536]{};

		float window_rms = 1.0f;

		NoiseSuppressorState() = default;
		~NoiseSuppressorState() { release_fft_plans(); }
		NoiseSuppressorState(const NoiseSuppressorState&) = delete;
		NoiseSuppressorState& operator=(const NoiseSuppressorState&) = delete;
		NoiseSuppressorState(NoiseSuppressorState&&) = delete;
		NoiseSuppressorState& operator=(NoiseSuppressorState&&) = delete;

	  private:
		void release_fft_plans()
		{
			const auto ptr_in_buffer = [](const void* ptr, const unsigned char* buffer, size_t buffer_size) -> bool
			{
				const auto p = reinterpret_cast<const unsigned char*>(ptr);
				return (p >= buffer) && (p < (buffer + buffer_size));
			};

			if (kiss_config_fwd && !ptr_in_buffer(kiss_config_fwd, kiss_cfg_mem_fwd, sizeof(kiss_cfg_mem_fwd)))
			{
				free(kiss_config_fwd);
			}
			if (kiss_config_inv && !ptr_in_buffer(kiss_config_inv, kiss_cfg_mem_inv, sizeof(kiss_cfg_mem_inv)))
			{
				free(kiss_config_inv);
			}
			kiss_config_fwd = nullptr;
			kiss_config_inv = nullptr;
		}
	};

	struct NoiseSuppressorOutputs
	{
		// Debug/telemetry: RMS of learned noise floor across bins.
		float noise_floor_rms = 0.0f;
	};

	class NoiseSuppressor
	{
	  public:
		static void plan_fft(NoiseSuppressorState& state);
		static void build_window(NoiseSuppressorState& state);
		static void reset_state(NoiseSuppressorState& state);

		static void process_frame(const NoiseSuppressorConfig& config,
			NoiseSuppressorState& state,
			const AudioFrame& input,
			AudioFrame& output,
			bool& is_noise_only,
			NoiseSuppressorOutputs& debug_outputs);
	};
} // namespace robotick
