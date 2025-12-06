// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#include "robotick/api.h"
#include "robotick/systems/audio/AudioFrame.h"
#include "robotick/systems/audio/AudioSystem.h"
#include "robotick/systems/auditory/HarmonicPitch.h"

#include <cstring>

namespace robotick
{
	struct HarmonicWaveGeneratorConfig
	{
		float amplitude_gain_db = 0.0f;
		int max_num_partials = 8;
	};

	struct HarmonicWaveGeneratorInputs
	{
		HarmonicPitchResult pitch_info;
	};

	struct HarmonicWaveGeneratorOutputs
	{
		AudioFrame mono;
	};

	struct HarmonicWaveGeneratorState
	{
		static constexpr int MaxOscillators = 1 + 16; // 1 base + up to 16 harmonics

		double phase[MaxOscillators] = {0.0};
		float prev_freq[MaxOscillators] = {0.0f};
		float prev_ampl[MaxOscillators] = {0.0f};
		double sample_accumulator = 0.0;
	};

	struct HarmonicWaveGeneratorWorkload
	{
		HarmonicWaveGeneratorConfig config;
		HarmonicWaveGeneratorInputs inputs;
		HarmonicWaveGeneratorOutputs outputs;
		State<HarmonicWaveGeneratorState> state;

		void load() { AudioSystem::init(); }

		void start(float) { outputs.mono.sample_rate = AudioSystem::get_sample_rate(); }

		void tick(const TickInfo& tick_info)
		{
			const int fs = outputs.mono.sample_rate;
			const double nyquist = 0.5 * fs;
			const double two_pi = 6.28318530717958647692;
			const float gain = powf(10.0f, config.amplitude_gain_db / 20.0f);

			const double exact_samples = static_cast<double>(fs) * tick_info.delta_time;
			state->sample_accumulator += exact_samples;
			int emit_samples = static_cast<int>(state->sample_accumulator);
			state->sample_accumulator -= emit_samples;

			if (emit_samples <= 0)
			{
				outputs.mono.samples.fill(0.0f);
				return;
			}

			emit_samples = robotick::min(emit_samples, static_cast<int>(outputs.mono.samples.capacity()));
			outputs.mono.samples.set_size(emit_samples);
			outputs.mono.samples.fill(0.0f);

			const HarmonicPitchResult& pitch_info = inputs.pitch_info;
			const float f0 = pitch_info.h1_f0_hz;

			if (f0 <= 0.0f || pitch_info.harmonic_amplitudes.empty())
			{
				return;
			}

			const int max_partials = robotick::clamp(config.max_num_partials, 0, HarmonicWaveGeneratorState::MaxOscillators - 1);
			const int num_partials = robotick::min(static_cast<int>(pitch_info.harmonic_amplitudes.size()), max_partials);

			for (int harmonic_index = 0; harmonic_index < num_partials; ++harmonic_index)
			{
				const int osc_index = harmonic_index; // 0 = h1, 1 = h2, etc.
				const float frequency = (harmonic_index + 1) * f0;
				if (frequency >= nyquist)
				{
					continue;
				}

				const float amplitude = pitch_info.harmonic_amplitudes[harmonic_index] * gain;

				float f0_prev = state->prev_freq[osc_index];
				float a0_prev = state->prev_ampl[osc_index];

				double phase = state->phase[osc_index];

				for (int i = 0; i < emit_samples; ++i)
				{
					const float t = (emit_samples > 1) ? static_cast<float>(i) / static_cast<float>(emit_samples - 1) : 0.0f;
					const float freq = f0_prev + t * (frequency - f0_prev);
					const float amp = a0_prev + t * (amplitude - a0_prev);
					outputs.mono.samples[i] += amp * sin(phase);
					phase += two_pi * freq / fs;
					if (phase >= two_pi)
					{
						phase -= two_pi;
					}
				}

				state->phase[osc_index] = phase;
				state->prev_freq[osc_index] = frequency;
				state->prev_ampl[osc_index] = amplitude;
			}
		}
	};

} // namespace robotick
