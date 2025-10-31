// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#include "robotick/api.h"
#include "robotick/systems/AudioBuffer.h"

#include <cmath>
#include <cstring>

namespace robotick
{

	// ======================================================
	// === SineWaveGeneratorWorkload ========================
	// ======================================================

	struct SineWaveGeneratorConfig
	{
		float frequency_hz = 440.0f;
		float amplitude = 0.1f;
		int samples_per_tick = 44; // ~44.1kHz @ 1000Hz tick
	};

	struct SineWaveGeneratorOutputs
	{
		AudioBuffer64 samples;
	};

	struct SineWaveGeneratorWorkload
	{
		SineWaveGeneratorOutputs outputs;
		SineWaveGeneratorConfig config;

		float phase = 0.0f;

		void setup() { outputs.samples.set_size(config.samples_per_tick); }

		void tick(const TickInfo& info)
		{
			const float sample_rate = info.tick_rate_hz * config.samples_per_tick;
			const float phase_step = 2.0f * 3.14159265f * config.frequency_hz / sample_rate;

			for (int i = 0; i < outputs.samples.size(); ++i)
			{
				outputs.samples[i] = config.amplitude * std::sin(phase);
				phase += phase_step;
				if (phase > 2.0f * 3.14159265f)
					phase -= 2.0f * 3.14159265f;
			}
		}
	};

} // namespace robotick
