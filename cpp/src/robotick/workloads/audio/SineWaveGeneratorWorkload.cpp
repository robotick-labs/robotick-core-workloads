// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#include "robotick/api.h"
#include "robotick/systems/audio/AudioBuffer.h"
#include "robotick/systems/audio/AudioSystem.h"

#include <cmath>
#include <cstring>

namespace robotick
{

	// ======================================================
	// === SineWaveGeneratorWorkload ========================
	// ======================================================

	struct SineWaveGeneratorInputs
	{
		float frequency_hz = 440.0f;
		float amplitude = 0.1f;
	};

	struct SineWaveGeneratorOutputs
	{
		AudioBuffer512 samples;
	};

	struct SineWaveGeneratorState
	{
		int sample_rate = 0;
		double samples_per_tick_exact = 0.0f;

		double phase = 0.0;
		double sample_accumulator = 0.0;
	};

	struct SineWaveGeneratorWorkload
	{
		SineWaveGeneratorInputs inputs;
		SineWaveGeneratorOutputs outputs;

		State<SineWaveGeneratorState> state;

		void load() { AudioSystem::init(); }

		void start(float tick_rate_hz)
		{
			state->sample_rate = AudioSystem::get_sample_rate();
			state->samples_per_tick_exact = static_cast<double>(state->sample_rate) / tick_rate_hz;
		}

		void tick(const TickInfo&)
		{
			// Accumulate fractional sample count based on exact samples-per-tick (e.g. 44.1)
			state->sample_accumulator += state->samples_per_tick_exact;

			// Emit integer number of samples this tick
			// (usually 44, sometimes 45 - since due to rounding of sample-rate we sometimes need a "leap tick")
			const int emit_samples = static_cast<int>(state->sample_accumulator);
			state->sample_accumulator -= emit_samples; // retain fractional remainder for next tick

			// Resize output buffer to match number of samples this tick
			outputs.samples.set_size(emit_samples);

			// Compute phase increment per sample
			const double phase_step = 2.0 * M_PI * inputs.frequency_hz / state->sample_rate;

			// Generate sine wave samples
			for (int i = 0; i < emit_samples; ++i)
			{
				outputs.samples[i] = static_cast<float>(inputs.amplitude * std::sin(state->phase));
				state->phase += phase_step;
				if (state->phase >= 2.0 * M_PI)
					state->phase -= 2.0 * M_PI;
			}
		}
	};

} // namespace robotick