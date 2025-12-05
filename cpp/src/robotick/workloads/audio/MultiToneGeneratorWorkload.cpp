// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#include "robotick/api.h"
#include "robotick/framework/math/Pow.h"
#include "robotick/framework/math/Trig.h"
#include "robotick/systems/audio/AudioFrame.h"
#include "robotick/systems/audio/AudioSystem.h"

#include <algorithm>
#include <cstring>

namespace robotick
{

	// ======================================================
	// === MultiToneGeneratorWorkload =======================
	// ======================================================

	struct ModulatedTone
	{
		float base_frequency_hz = 440.0f;
		float base_amplitude = 0.0f;
		float modulation_freq_hz = 0.0f;
		float modulation_depth_cents = 0.0f;
	};

	ROBOTICK_REGISTER_STRUCT_BEGIN(ModulatedTone)
	ROBOTICK_STRUCT_FIELD(ModulatedTone, float, base_frequency_hz)
	ROBOTICK_STRUCT_FIELD(ModulatedTone, float, base_amplitude)
	ROBOTICK_STRUCT_FIELD(ModulatedTone, float, modulation_freq_hz)
	ROBOTICK_STRUCT_FIELD(ModulatedTone, float, modulation_depth_cents)
	ROBOTICK_REGISTER_STRUCT_END(ModulatedTone)

	struct MultiToneGeneratorConfig
	{
		float amplitude_gain_db = 0.0f; // Linear gain multiplier = pow(10, amplitude_gain_db / 20)
	};

	struct MultiToneGeneratorInputs
	{
		ModulatedTone tone1;
		ModulatedTone tone2;
		ModulatedTone tone3;
		ModulatedTone tone4;
		ModulatedTone tone5;
	};

	struct MultiToneGeneratorOutputs
	{
		AudioFrame mono; // emit-size varies per tick (leap-tick aware)
	};

	struct ToneState
	{
		double phase = 0.0;
		double mod_phase = 0.0;
		float prev_frequency_hz = 0.0f;
		float prev_amplitude = 0.0f;
	};

	struct MultiToneGeneratorState
	{
		double sample_accumulator = 0.0;

		ToneState tone1;
		ToneState tone2;
		ToneState tone3;
		ToneState tone4;
		ToneState tone5;
	};

	struct MultiToneGeneratorWorkload
	{
		MultiToneGeneratorConfig config;
		MultiToneGeneratorInputs inputs;
		MultiToneGeneratorOutputs outputs;
		State<MultiToneGeneratorState> state;

		void load() { AudioSystem::init(); }

		void start(float /*tick_rate_hz*/) { outputs.mono.sample_rate = AudioSystem::get_sample_rate(); }

		void tick(const TickInfo& tick_info)
		{
			const int fs = outputs.mono.sample_rate;
			const double nyquist = 0.5 * fs;
			const double two_pi = 6.28318530717958647692;
			const float gain = robotick::pow(10.0f, config.amplitude_gain_db / 20.0f);

			const double exact_samples_this_tick = (double)fs * (double)tick_info.delta_time;
			state->sample_accumulator += exact_samples_this_tick;
			int emit_samples = (int)state->sample_accumulator;
			state->sample_accumulator -= emit_samples;

			if (emit_samples <= 0)
			{
				outputs.mono.samples.fill(0.0f);
				return;
			}

			emit_samples = robotick::min(emit_samples, (int)outputs.mono.samples.capacity());
			outputs.mono.samples.set_size(emit_samples);
			outputs.mono.samples.fill(0.0f); // Will accumulate tones

			auto emit_modulated_tone = [&](const ModulatedTone& tone, ToneState& tone_state)
			{
				if (tone.base_amplitude <= 0.0f || tone.base_frequency_hz <= 0.0f)
					return;

				const double f0 = tone_state.prev_frequency_hz;
				const double f1 = clamp(tone.base_frequency_hz, 0.0f, (float)(nyquist - 1.0));
				const double a0 = tone_state.prev_amplitude * gain;
				const double a1 = tone.base_amplitude * gain;

				const double mod_freq = clamp((double)tone.modulation_freq_hz, 0.0, nyquist - 1.0);
				const double mod_depth_cents = tone.modulation_depth_cents;

				tone_state.prev_frequency_hz = f1;
				tone_state.prev_amplitude = tone.base_amplitude;

				double phase = tone_state.phase;
				double mod_phase = tone_state.mod_phase;

				for (int i = 0; i < emit_samples; ++i)
				{
					const double t = (double)i / (emit_samples - 1);
					const double amp = a0 + (a1 - a0) * t;
					const double base_freq = f0 + (f1 - f0) * t;

					double mod_multiplier = 1.0;
					if (mod_freq > 0.0 && mod_depth_cents != 0.0)
					{
						const double mod_sin = robotick::sin(mod_phase);
						mod_multiplier = robotick::pow(2.0, (mod_sin * mod_depth_cents) / 1200.0);
					}

					const double freq = base_freq * mod_multiplier;
					const double step = two_pi * freq / fs;

					outputs.mono.samples[i] += (float)(amp * robotick::sin(phase));

					phase += step;
					mod_phase += two_pi * mod_freq / fs;

					if (phase >= two_pi)
						phase -= two_pi;
					if (mod_phase >= two_pi)
						mod_phase -= two_pi;
				}

				tone_state.phase = phase;
				tone_state.mod_phase = mod_phase;
			};

			emit_modulated_tone(inputs.tone1, state->tone1);
			emit_modulated_tone(inputs.tone2, state->tone2);
			emit_modulated_tone(inputs.tone3, state->tone3);
			emit_modulated_tone(inputs.tone4, state->tone4);
			emit_modulated_tone(inputs.tone5, state->tone5);
		}
	};

} // namespace robotick
