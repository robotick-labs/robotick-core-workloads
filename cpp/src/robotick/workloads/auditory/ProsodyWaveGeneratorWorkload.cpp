// Copyright Robotick
// SPDX-License-Identifier: Apache-2.0

#include "robotick/api.h"
#include "robotick/systems/audio/AudioBuffer.h"
#include "robotick/systems/audio/AudioSystem.h"
#include "robotick/systems/auditory/ProsodyState.h"

#include <algorithm>
#include <cmath>

namespace robotick
{
	// ======================================================
	// === ProsodyWaveGeneratorWorkload =====================
	// ======================================================

	struct ProsodyWaveGeneratorConfig
	{
		// Overall gain applied in dB (linear = pow(10, dB/20))
		float amplitude_gain_db = 0.0f;

		// Harmonically voiced tone settings
		int num_harmonics = 4;		   // 0..8 recommended
		float harmonic_rolloff = 0.6f; // 0..1 (per harmonic multiplier, e.g. amp *= rolloff^n)

		// Noise bed (unvoiced/air/brightness)
		// Base mix from spectral_flatness via: mix = clamp(flatness^flatness_gamma, 0..1)
		float flatness_gamma = 1.0f; // 0.5..2.0 typical

		// Simple noise shaping: set a 1-pole LPF cutoff as:
		// cutoff_hz = clamp( centroid + bandwidth_scale * bandwidth, 80..fs/2-1 )
		float bandwidth_scale = 0.5f;

		// If true, when voiced==true and pitch_hz>0, prefer pitch;
		// otherwise fall back to centroid-based pitch guess.
		bool prefer_pitch_when_voiced = true;

		// Safety: minimum audible frequency when falling back
		float min_fallback_hz = 80.0f;
	};

	struct ProsodyWaveGeneratorInputs
	{
		// The prosody features for this tick (already computed by ProsodyAnalyserWorkload)
		ProsodyState prosody_state;
	};

	struct ProsodyWaveGeneratorOutputs
	{
		AudioBuffer512 mono;
	};

	struct ProsodyWaveGeneratorState
	{
		int sample_rate = 44100;

		// fractional sample accumulator (leap-tick)
		double sample_accum = 0.0;

		// oscillator phases: fundamental + a few harmonics (2..N)
		// Store up to 9 entries (index 0 = fundamental). We'll use num_harmonics+1 slots.
		static constexpr int MaxOsc = 9;
		double phase[MaxOsc] = {0.0};

		// previous block targets for de-zippering
		float prev_carrier_hz = 220.0f;
		float prev_amp_linear = 0.0f;

		// noise filter state (simple 1-pole low-pass)
		float noise_z1 = 0.0f;

		// tiny PRNG (xorshift32)
		uint32_t rng = 0x12345678u;
		inline float noise_uniform_pm1()
		{
			uint32_t x = rng;
			x ^= x << 13;
			x ^= x >> 17;
			x ^= x << 5;
			rng = x;
			// map to [-1,1]
			return (float)((int32_t)(x) / 2147483648.0); // ≈ [-1,1)
		}
	};

	struct ProsodyWaveGeneratorWorkload
	{
		ProsodyWaveGeneratorConfig config;
		ProsodyWaveGeneratorInputs inputs;
		ProsodyWaveGeneratorOutputs outputs;
		State<ProsodyWaveGeneratorState> state;

		void load() { AudioSystem::init(); }

		void start(float /*tick_rate_hz*/) { state->sample_rate = AudioSystem::get_sample_rate(); }

		void tick(const TickInfo& info)
		{
			const int fs = state->sample_rate;
			const double nyquist = 0.5 * fs;
			const double two_pi = 6.28318530717958647692;

			// --- Map prosody → synthesis controls ---
			const auto& p = inputs.prosody_state;

			// Loudness from RMS (already linear), then apply global dB gain:
			const float gain = inputs.prosody_state.voiced ? std::pow(10.0f, config.amplitude_gain_db / 20.0f) : 0.0f;
			const float target_amp_lin = std::max(0.0f, p.rms) * gain; // keep overall loudness tied to RMS only

			// Pick carrier frequency
			float carrier_hz = 0.0f;
			if (config.prefer_pitch_when_voiced && p.voiced && p.pitch_hz > 0.0f)
			{
				carrier_hz = p.pitch_hz;
			}
			else
			{
				// fallback from spectral centroid (brightness → perceived pitch-ish)
				// Heuristic: centroid tends to be > F0, so bias it down a bit.
				float guess = p.spectral_centroid_hz * 0.75f;
				if (!(guess > 0.0f))
					guess = std::max(config.min_fallback_hz, 0.5f * (float)fs / 512.0f); // sane fallback
				carrier_hz = guess;
			}

			// Safety clamp to avoid aliasing
			carrier_hz = std::clamp(carrier_hz, 0.0f, (float)(nyquist - 1.0));

			// Tone-vs-noise base mix from flatness (0=tonal, 1=noisy)
			float flat = std::clamp(p.spectral_flatness, 0.0f, 1.0f);
			if (config.flatness_gamma != 1.0f)
				flat = std::pow(flat, std::max(0.1f, config.flatness_gamma));
			float base_noise = flat;			 // 0..1
			float base_tone = 1.0f - base_noise; // 0..1

			// --- energy-driven spectral shaping (does NOT change overall loudness) ---
			// Use normalized spectral energy to decide how "tonal" vs "noisy" to sound.
			const float energy_ratio = std::clamp(p.spectral_energy_ratio, 0.0f, 2.0f); // 1.0 = neutral
			// Bias: more spectral energy → more tone; less → more noise.
			float tone_biased = base_tone * energy_ratio;
			float noise_biased = base_noise * (2.0f - energy_ratio);
			const float mix_sum = std::max(1e-6f, tone_biased + noise_biased);
			const float tone_mix = tone_biased / mix_sum;
			const float noise_mix = noise_biased / mix_sum;

			// Make harmonic ladder track energy (but not overall amplitude)
			const float harmonic_energy_gain = std::clamp(energy_ratio, 0.5f, 1.5f);

			// Simple noise shaping cutoff using centroid + bandwidth,
			// then tilt slightly with energy_ratio (more tonal → slightly darker noise).
			float cutoff_hz = p.spectral_centroid_hz + config.bandwidth_scale * p.spectral_bandwidth_hz;
			{
				const float tilt = (energy_ratio - 1.0f) * 0.25f; // ±25% tilt window
				const float tilt_scale = std::clamp(1.0f - tilt, 0.7f, 1.3f);
				cutoff_hz *= tilt_scale;
			}
			cutoff_hz = std::clamp(cutoff_hz, 80.0f, (float)(nyquist - 1.0));

			// --- Compute how many samples to emit this tick (leap-tick safe) ---
			state->sample_accum += (double)fs * (double)info.delta_time;
			int emit = (int)state->sample_accum;
			state->sample_accum -= emit;

			if (emit <= 0)
			{
				outputs.mono.set_size(0);
				// still update previous targets for smooth re-entry
				state->prev_amp_linear = target_amp_lin;
				state->prev_carrier_hz = carrier_hz;
				return;
			}

			emit = std::min(emit, (int)outputs.mono.capacity());
			outputs.mono.set_size(emit);

			// --- Prepare interpolation (de-zipper) across the block ---
			const float a0 = state->prev_amp_linear;
			const float a1 = target_amp_lin;
			const float f0 = state->prev_carrier_hz;
			const float f1 = carrier_hz;

			// Limit harmonics
			const int H = std::clamp(config.num_harmonics, 0, ProsodyWaveGeneratorState::MaxOsc - 1);

			// Noise filter coefficient: simple 1-pole low-pass
			// alpha = 1 - exp(-2π * fc / fs), yields stable 0..1
			const float alpha = 1.0f - std::exp(-2.0f * (float)M_PI * (cutoff_hz / (float)fs));

			// Generate
			double phases[ProsodyWaveGeneratorState::MaxOsc];
			for (int i = 0; i <= H; ++i)
				phases[i] = state->phase[i];

			float z1 = state->noise_z1;

			if (emit == 1)
			{
				const double amp = (double)a1;
				const double freq = (double)f1;

				// Tone: fundamental + harmonics
				double tone = 0.0;
				if (freq > 0.0)
				{
					// fundamental
					double step = two_pi * freq / (double)fs;
					tone += std::sin(phases[0]);
					phases[0] += step;

					// harmonics (energy-shaped)
					double h_amp = (double)config.harmonic_rolloff * (double)harmonic_energy_gain;
					for (int h = 1; h <= H; ++h)
					{
						const double fhz = freq * (double)(h + 1);
						if (fhz >= nyquist)
							break;
						const double hstep = two_pi * fhz / (double)fs;
						tone += h_amp * std::sin(phases[h]);
						phases[h] += hstep;
						h_amp *= (double)config.harmonic_rolloff;
					}
				}

				// Noise: white → 1-pole LPF
				float w = state->noise_uniform_pm1();
				z1 = z1 + alpha * (w - z1);

				const double s = (double)tone_mix * tone; // + (double)noise_mix * (double)z1;

				outputs.mono[0] = (float)(amp * s);
			}
			else
			{
				for (int i = 0; i < emit; ++i)
				{
					const double t = (double)i / (double)(emit - 1);
					const double amp = (double)a0 + (double)(a1 - a0) * t;
					const double freq = (double)f0 + (double)(f1 - f0) * t;

					// Tone
					double tone = 0.0;
					if (freq > 0.0)
					{
						// fundamental
						const double step0 = two_pi * freq / (double)fs;
						tone += std::sin(phases[0]);
						phases[0] += step0;

						// harmonics (energy-shaped)
						double h_amp = (double)config.harmonic_rolloff * (double)harmonic_energy_gain;
						for (int h = 1; h <= H; ++h)
						{
							const double fhz = freq * (double)(h + 1);
							if (fhz >= nyquist)
								break;
							const double hstep = two_pi * fhz / (double)fs;
							tone += h_amp * std::sin(phases[h]);
							phases[h] += hstep;
							h_amp *= (double)config.harmonic_rolloff;
						}
					}

					// Noise (LPF’d white)
					float w = state->noise_uniform_pm1();
					z1 = z1 + alpha * (w - z1);

					const double s = (double)tone_mix * tone + (double)noise_mix * (double)z1;
					outputs.mono[i] = (float)(amp * s);

					// Wrap phases
					for (int h = 0; h <= H; ++h)
					{
						if (phases[h] >= two_pi)
							phases[h] -= two_pi;
						else if (phases[h] < 0.0)
							phases[h] += two_pi;
					}
				}
			}

			// persist
			for (int h = 0; h <= H; ++h)
				state->phase[h] = phases[h];
			state->noise_z1 = z1;
			state->prev_amp_linear = target_amp_lin;
			state->prev_carrier_hz = carrier_hz;
		}
	};

} // namespace robotick
