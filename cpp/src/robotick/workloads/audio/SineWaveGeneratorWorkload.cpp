// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#include "robotick/api.h"
#include "robotick/framework/math/Pow.h"
#include "robotick/systems/audio/AudioFrame.h"
#include "robotick/systems/audio/AudioSystem.h"

#include <algorithm>
#include <cstring>

namespace robotick
{

	// ======================================================
	// === SineWaveGeneratorWorkload ========================
	// ======================================================

	struct SineWaveGeneratorConfig
	{
		float amplitude_gain_db = 0.0f; // Linear gain multiplier = pow(10, amplitude_gain_db / 20)
	};

	struct SineWaveGeneratorInputs
	{
		float frequency_hz = 440.0f;
		float amplitude = 0.1f;
	};

	struct SineWaveGeneratorOutputs
	{
		AudioFrame mono; // emit-size varies per tick (leap-tick aware)
	};

	struct SineWaveGeneratorState
	{
		// Fractional “leap-tick” accumulator (handles non-integer samples/tick)
		double sample_accumulator = 0.0;

		// Continuous oscillator phase (radians)
		double phase = 0.0;

		// Previous controls (for per-block linear ramp)
		float prev_frequency_hz = 440.0f;
		float prev_amplitude = 0.1f;
	};

	struct SineWaveGeneratorWorkload
	{
		SineWaveGeneratorConfig config;
		SineWaveGeneratorInputs inputs;
		SineWaveGeneratorOutputs outputs;
		State<SineWaveGeneratorState> state;

		void load() { AudioSystem::init(); }

		void start(float /*tick_rate_hz*/) { outputs.mono.sample_rate = AudioSystem::get_sample_rate(); }

		void tick(const TickInfo& tick_info)
		{
			const int fs = outputs.mono.sample_rate;
			const double nyquist = 0.5 * fs;
			const double two_pi = 6.28318530717958647692;

			// Clamp inputs safely (no throws)
			const float target_amp = max(0.0f, inputs.amplitude);
			const float target_freq = clamp(inputs.frequency_hz, 0.0f, (float)(nyquist - 1.0));

			// Apply global gain factor from config
			const float gain = robotick::pow(10.0f, config.amplitude_gain_db / 20.0f);
			const float scaled_a0 = state->prev_amplitude * gain;
			const float scaled_a1 = target_amp * gain;

			// Always update prev values, even on silent ticks
			const float f0 = state->prev_frequency_hz;
			const float f1 = target_freq;
			state->prev_amplitude = target_amp;
			state->prev_frequency_hz = target_freq;

			// Early out if silent
			if (scaled_a1 <= 0.0f || f1 <= 0.0f)
			{
				outputs.mono.samples.fill(0.0f);
				return;
			}

			// Sample count this tick (leap-tick aware)
			const double exact_samples_this_tick = (double)fs * (double)tick_info.delta_time;
			state->sample_accumulator += exact_samples_this_tick;
			int emit_samples = (int)state->sample_accumulator;
			state->sample_accumulator -= emit_samples;

			if (emit_samples <= 0)
			{
				outputs.mono.samples.fill(0.0f);
				return;
			}

			emit_samples = min(emit_samples, (int)outputs.mono.samples.capacity());
			outputs.mono.samples.set_size(emit_samples);

			double phase = state->phase;

			if (emit_samples == 1)
			{
				const double step = two_pi * (double)f1 / (double)fs;
				outputs.mono.samples[0] = (float)(scaled_a1 * std::sin(phase));
				phase += step;
				if (phase >= two_pi)
					phase -= two_pi;
			}
			else
			{
				for (int i = 0; i < emit_samples; ++i)
				{
					const double t = (double)i / (double)(emit_samples - 1);
					const double amp = (double)scaled_a0 + (double)(scaled_a1 - scaled_a0) * t;
					const double freq = (double)f0 + (double)(f1 - f0) * t;
					const double step = two_pi * freq / (double)fs;

					outputs.mono.samples[i] = (float)(amp * std::sin(phase));
					phase += step;

					if (phase >= two_pi)
						phase -= two_pi;
					else if (phase < 0.0)
						phase += two_pi;
				}
			}

			state->phase = phase;
		}
	};

} // namespace robotick
