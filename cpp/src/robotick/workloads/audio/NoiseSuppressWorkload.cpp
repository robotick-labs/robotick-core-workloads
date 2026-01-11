// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#include "robotick/api.h"
#include "robotick/systems/audio/AudioFrame.h"
#include "robotick/systems/audio/NoiseSuppressor.h"

namespace robotick
{
	struct NoiseSuppressConfig
	{
		NoiseSuppressorConfig settings; // Noise suppression settings (profile learning + attenuation behavior)
	};

	struct NoiseSuppressInputs
	{
		AudioFrame mono;
	};

	struct NoiseSuppressOutputs
	{
		// Denoised audio + a noise-only hint for downstream workloads.
		AudioFrame mono;
		bool is_noise_only = false;
		float noise_floor_rms = 0.0f;
	};

	struct NoiseSuppressState
	{
		NoiseSuppressorState suppressor_state;
		NoiseSuppressorOutputs debug_outputs;
		bool is_initialized = false;
	};

	struct NoiseSuppressWorkload
	{
		NoiseSuppressConfig config;
		NoiseSuppressInputs inputs;
		NoiseSuppressOutputs outputs;
		StatePtr<NoiseSuppressState> state;

		void load()
		{
			// One-time setup for FFT plans and state buffers.
			NoiseSuppressor::plan_fft(state->suppressor_state);
			NoiseSuppressor::build_window(state->suppressor_state);
			NoiseSuppressor::reset_state(state->suppressor_state);
			state->is_initialized = true;
		}

		void tick(const TickInfo&)
		{
			if (!state->is_initialized)
			{
				load();
			}

			// Apply suppression to the incoming audio block and publish metadata.
			NoiseSuppressor::process_frame(
				config.settings, state->suppressor_state, inputs.mono, outputs.mono, outputs.is_noise_only, state->debug_outputs);

			outputs.noise_floor_rms = state->debug_outputs.noise_floor_rms;
		}
	};
} // namespace robotick
