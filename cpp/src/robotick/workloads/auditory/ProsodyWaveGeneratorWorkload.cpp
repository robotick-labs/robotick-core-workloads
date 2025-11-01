// Copyright Robotick
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "robotick/api.h"
#include "robotick/systems/audio/AudioBuffer.h"
#include "robotick/systems/audio/AudioSystem.h"
#include "robotick/systems/auditory/ProsodyState.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>

namespace robotick
{
	// ======================================================
	// ProsodyWaveGeneratorWorkload — uses ProsodyState fields
	// ======================================================

	struct ProsodyWaveGeneratorConfig
	{
		// --- Output level ---
		float amplitude_gain_db = 0.0f; // linear gain = 10^(dB/20)

		// --- Synthesis shape ---
		int num_harmonics = 4;		   // 0..8
		float harmonic_rolloff = 0.6f; // geometric per-harmonic gain
		float min_fallback_hz = 80.0f;

		// --- Spectral → noise shaping / mix ---
		float flatness_gamma = 1.0f;  // curve for tone/noise from flatness
		float bandwidth_scale = 0.5f; // LPF cutoff from centroid + scale*bandwidth

		// --- Synthesis toggles (sound-building blocks) ---
		bool enable_tone = true;	  // enable sinusoidal component
		bool enable_harmonics = true; // add harmonic ladder on top of fundamental
		bool enable_noise = true;	  // add LPF white noise bed

		// --- ProsodyState usage toggles (what to *use* from inputs) ---
		bool use_voiced_gate = true;	   // respect prosody.voiced (no output when false)
		bool use_rms_for_amplitude = true; // scale loudness by prosody.rms (then dB gain)
		bool use_pitch_hz = true;		   // use prosody.pitch_hz for carrier

		// Bulk spectral switch (gates all spectral fields):
		bool use_spectral_features = true;

		// Fine spectral toggles (also require use_spectral_features):
		bool use_spectral_centroid = true;	// centroid as pitch fallback + noise cutoff base
		bool use_spectral_bandwidth = true; // bandwidth contribution to noise cutoff
		bool use_spectral_flatness = true;	// tone/noise mixing from flatness
		bool use_spectral_energy = true;	// tone/noise bias from spectral_energy_ratio
	};

	struct ProsodyWaveGeneratorInputs
	{
		ProsodyState prosody_state;
	};

	struct ProsodyWaveGeneratorOutputs
	{
		AudioBuffer512 mono;
	};

	struct ProsodyWaveGeneratorState
	{
		static constexpr int MaxOsc = 9;

		// system
		int sample_rate = 44100;
		double sample_accum = 0.0;

		// oscillator phases
		double phase[MaxOsc] = {0.0};

		// noise filter
		float noise_z1 = 0.0f;

		// previous targets
		float prev_amp_linear = 0.0f;

		// smoothed mappings
		float smoothed_pitch_hz = 0.0f;
		float tone_mix_z = 1.0f;
		float noise_mix_z = 0.0f;

		// PRNG
		uint32_t rng = 0x12345678u;
		inline float noise_uniform_pm1()
		{
			uint32_t x = rng;
			x ^= x << 13;
			x ^= x >> 17;
			x ^= x << 5;
			rng = x;
			return (float)((int32_t)(x) / 2147483648.0f); // [-1,1)
		}
	};

	struct ProsodyWaveGeneratorWorkload
	{
		ProsodyWaveGeneratorConfig config;
		ProsodyWaveGeneratorInputs inputs;
		ProsodyWaveGeneratorOutputs outputs;
		State<ProsodyWaveGeneratorState> state;

		void load()
		{
			AudioSystem::init();
			std::fill(std::begin(state->phase), std::end(state->phase), 0.0);
			state->noise_z1 = 0.0f;
			state->prev_amp_linear = 0.0f;
			state->smoothed_pitch_hz = 0.0f;
			state->tone_mix_z = 1.0f;
			state->noise_mix_z = 0.0f;
			state->sample_accum = 0.0;
		}

		void start(float /*tick_rate_hz*/) { state->sample_rate = AudioSystem::get_sample_rate(); }

		void tick(const TickInfo& info)
		{
			const auto& p = inputs.prosody_state;
			const int fs = state->sample_rate;
			const double nyq = 0.5 * (double)fs;
			const double guard = 0.98 * nyq; // avoid alias at Nyquist
			const double two_pi = 6.28318530717958647692;

			// === Gate by voiced (if enabled) ===
			if (config.use_voiced_gate && !p.voiced)
			{
				outputs.mono.set_size(0);
				state->prev_amp_linear = 0.0f; // silence
				return;
			}

			// === Amplitude ===
			const float lin_gain = std::pow(10.0f, config.amplitude_gain_db / 20.0f);
			float target_amp = lin_gain;
			if (config.use_rms_for_amplitude)
				target_amp *= std::max(0.0f, p.rms);

			// === Carrier frequency selection ===
			float carrier_hz = 0.0f;

			if (config.use_pitch_hz && p.pitch_hz > 0.0f)
			{
				carrier_hz = p.pitch_hz;
			}
			else
			{
				float fallback = config.min_fallback_hz;

				// If allowed, nudge fallback from spectral centroid (typically > F0 → damp it)
				if (config.use_spectral_features && config.use_spectral_centroid && p.spectral_centroid_hz > 0.0f)
				{
					fallback = std::max(fallback, p.spectral_centroid_hz * 0.75f);
				}

				carrier_hz = fallback;
			}

			carrier_hz = std::clamp(carrier_hz, 0.0f, (float)(nyq - 1.0));

			// Smooth pitch across ticks to avoid squeaks from frame jitter
			{
				const float alpha = 0.20f;
				if (state->smoothed_pitch_hz <= 0.0f)
					state->smoothed_pitch_hz = carrier_hz;
				state->smoothed_pitch_hz = (1.0f - alpha) * state->smoothed_pitch_hz + alpha * carrier_hz;
			}
			const double freq_tick = (double)state->smoothed_pitch_hz;

			// === Tone/Noise mix from spectral features ===
			float tone_mix = 1.0f;
			float noise_mix = 0.0f;

			if (config.use_spectral_features && config.use_spectral_flatness)
			{
				float flat = std::clamp(p.spectral_flatness, 0.0f, 1.0f);
				if (config.flatness_gamma != 1.0f)
					flat = std::pow(flat, std::max(0.1f, config.flatness_gamma));
				tone_mix = 1.0f - flat;
				noise_mix = flat;
			}

			if (config.use_spectral_features && config.use_spectral_energy)
			{
				float er = std::clamp(p.spectral_energy_ratio, 0.0f, 2.0f); // ~1 neutral
				tone_mix *= er;
				noise_mix *= (2.0f - er);
				const float sum = std::max(1e-6f, tone_mix + noise_mix);
				tone_mix /= sum;
				noise_mix /= sum;
			}

			// Honor synthesis block toggles
			if (!config.enable_tone && config.enable_noise)
			{
				tone_mix = 0.0f;
				noise_mix = 1.0f;
			}
			if (!config.enable_noise)
			{
				noise_mix = 0.0f;
			}
			if (!config.enable_tone && !config.enable_noise)
			{
				tone_mix = 1.0f;
			} // fail-safe to something audible

			// Slew mix for stability
			{
				const float mix_alpha = 0.20f;
				state->tone_mix_z = (1.0f - mix_alpha) * state->tone_mix_z + mix_alpha * tone_mix;
				state->noise_mix_z = (1.0f - mix_alpha) * state->noise_mix_z + mix_alpha * noise_mix;
				tone_mix = state->tone_mix_z;
				noise_mix = state->noise_mix_z;
			}

			// === Noise filter cutoff from spectral centroid/bandwidth ===
			float cutoff_hz = 2000.0f; // sensible default
			if (config.use_spectral_features)
			{
				float base = (config.use_spectral_centroid ? p.spectral_centroid_hz : 0.0f);
				float bw = (config.use_spectral_bandwidth ? p.spectral_bandwidth_hz : 0.0f);
				if (base <= 0.0f)
					base = 1000.0f; // fallback mid-bright
				cutoff_hz = base + config.bandwidth_scale * bw;
			}
			cutoff_hz = std::clamp(cutoff_hz, 80.0f, (float)(nyq - 1.0));
			float alpha = 1.0f - std::exp(-2.0f * (float)M_PI * (cutoff_hz / (float)fs));
			alpha = std::clamp(alpha, 1e-5f, 0.9999f);

			// === Sample budget this tick ===
			state->sample_accum += (double)fs * (double)info.delta_time;
			int count = (int)state->sample_accum;
			state->sample_accum -= count;

			if (count <= 0)
			{
				outputs.mono.set_size(0);
				state->prev_amp_linear = target_amp;
				return;
			}

			count = std::min(count, (int)outputs.mono.capacity());
			outputs.mono.set_size(count);

			// === Interpolate amplitude across block (freq constant per tick) ===
			const float amp0 = state->prev_amp_linear;
			const float amp1 = target_amp;

			double local_phase[ProsodyWaveGeneratorState::MaxOsc];
			std::memcpy(local_phase, state->phase, sizeof(local_phase));
			float z1 = state->noise_z1;

			const int H =
				(config.enable_tone && config.enable_harmonics) ? std::clamp(config.num_harmonics, 0, ProsodyWaveGeneratorState::MaxOsc - 1) : 0;

			const double step_fund = two_pi * std::clamp(freq_tick, 0.0, guard) / (double)fs;
			const double denom = (count > 1) ? (double)(count - 1) : 1.0;

			for (int i = 0; i < count; ++i)
			{
				const double t = (double)i / denom;
				const double amp = (double)amp0 + ((double)amp1 - (double)amp0) * t;

				double s = 0.0;

				// --- Tone ---
				if (config.enable_tone && freq_tick > 0.0)
				{
					// fundamental
					s += std::sin(local_phase[0]);
					local_phase[0] += step_fund;

					// harmonics
					if (H > 0)
					{
						double h_amp = (double)config.harmonic_rolloff;
						for (int h = 1; h <= H; ++h)
						{
							const double hf = freq_tick * (double)(h + 1);
							if (hf >= guard)
								break;
							const double hstep = two_pi * hf / (double)fs;
							s += h_amp * std::sin(local_phase[h]);
							local_phase[h] += hstep;
							h_amp *= (double)config.harmonic_rolloff;
						}
					}
				}

				// --- Noise ---
				if (config.enable_noise)
				{
					const float white = state->noise_uniform_pm1();
					z1 = z1 + alpha * (white - z1);
					if (std::fabs(z1) < 1e-20f)
						z1 = 0.0f; // denormal guard

					s = (double)tone_mix * s + (double)noise_mix * (double)z1;
				}

				outputs.mono[i] = (float)(amp * s);

				// Phase wrap (light)
				for (int h = 0; h <= H; ++h)
				{
					if (local_phase[h] >= two_pi)
						local_phase[h] -= two_pi;
					else if (local_phase[h] < 0.0)
						local_phase[h] += two_pi;
				}
			}

			// persist
			for (int h = 0; h <= H; ++h)
				state->phase[h] = local_phase[h];
			state->noise_z1 = z1;
			state->prev_amp_linear = target_amp;
		}
	};

} // namespace robotick
