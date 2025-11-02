// Copyright Robotick
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "robotick/api.h"
#include "robotick/systems/audio/AudioFrame.h"
#include "robotick/systems/audio/AudioSystem.h"
#include "robotick/systems/auditory/ProsodyState.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>

namespace robotick
{
	struct ProsodyWaveGeneratorConfig
	{
		// --- Global output ---
		float amplitude_gain_db = 0.0f;
		bool use_rms_for_amplitude = true;
		bool use_voiced_gate = true;

		// --- Pitch selection ---
		// If true, we generate tone/partials only when a measured p.pitch_hz > 0.
		bool use_pitch_hz = true;

		// --- Tone (fundamental) ---
		bool enable_tone = true;
		float tone_base = 1.0f;
		bool use_flatness_for_tone = true;
		bool use_energy_ratio_for_tone = true;
		float tone_energy_ratio_center = 1.0f;
		float tone_energy_ratio_scale = 1.0f;

		// --- Partials (from analyser only; no synthetic fallback) ---
		bool enable_partials = true;
		float partials_base = 0.6f;

		// --- Noise ---
		bool enable_noise = true;
		float noise_base = 0.5f;
		bool use_flatness_for_noise = true;
		bool use_energy_ratio_for_noise = true;
		float noise_energy_ratio_center = 1.0f;
		float noise_energy_ratio_scale = 1.0f;

		// --- Noise coloration ---
		bool use_spectral_for_noise_lpf = true;
		float bandwidth_scale = 0.5f;
		float noise_cutoff_default_hz = 2000.0f;

		// --- Smoothing ---
		float pitch_smooth_alpha = 0.20f; // only applied when p.pitch_hz > 0
		float mix_smooth_alpha = 0.20f;

		// --- Safety ---
		float min_component_gain = 0.0f;
		float max_component_gain = 2.0f;
	};

	struct ProsodyWaveGeneratorInputs
	{
		ProsodyState prosody_state;
	};

	struct ProsodyWaveGeneratorOutputs
	{
		AudioFrame mono;
	};

	struct ProsodyWaveGeneratorState
	{
		static constexpr int MaxOsc = 1 + Prosody::MaxPartials; // 1 fund + partials

		double sample_accum = 0.0;

		double phase[MaxOsc] = {0.0};
		float noise_z1 = 0.0f;

		float prev_amp_linear = 0.0f;
		float smoothed_pitch_hz = 0.0f;

		float tone_gain_z = 0.0f;
		float part_gain_z = 0.0f;
		float noise_gain_z = 0.0f;

		uint32_t rng = 0x12345678u;
		inline float noise_uniform_pm1()
		{
			uint32_t x = rng;
			x ^= x << 13;
			x ^= x >> 17;
			x ^= x << 5;
			rng = x;
			return (float)((int32_t)x / 2147483648.0f);
		}
	};

	struct ProsodyWaveGeneratorWorkload
	{
		ProsodyWaveGeneratorConfig config;
		ProsodyWaveGeneratorInputs inputs;
		ProsodyWaveGeneratorOutputs outputs;
		State<ProsodyWaveGeneratorState> state;

		static inline float clamp01(float v) { return v < 0 ? 0.f : (v > 1 ? 1.f : v); }

		void load()
		{
			AudioSystem::init();
			std::fill(std::begin(state->phase), std::end(state->phase), 0.0);
			state->noise_z1 = 0.0f;
			state->prev_amp_linear = 0.0f;
			state->smoothed_pitch_hz = 0.0f;
			state->tone_gain_z = state->part_gain_z = state->noise_gain_z = 0.0f;
			state->sample_accum = 0.0;
		}

		void start(float) { outputs.mono.sample_rate = AudioSystem::get_sample_rate(); }

		void tick(const TickInfo& tick_info)
		{
			static constexpr double ns_to_sec = 1e-9;
			outputs.mono.timestamp = ns_to_sec * (double)tick_info.time_now_ns;

			const auto& p = inputs.prosody_state;
			const int fs = outputs.mono.sample_rate;
			const double nyq = 0.5 * (double)fs;
			const double guard = 0.98 * nyq;
			const double two_pi = 6.28318530717958647692;

			// Gate: if requested, output nothing when not voiced
			if (config.use_voiced_gate && !p.voiced)
			{
				outputs.mono.samples.clear();
				state->prev_amp_linear = 0.0f;
				// Also reset smoothed pitch to avoid carry-over
				state->smoothed_pitch_hz = 0.0f;
				return;
			}

			// Global amplitude
			float lin_gain = std::pow(10.0f, config.amplitude_gain_db / 20.0f);
			if (config.use_rms_for_amplitude)
				lin_gain *= std::max(0.0f, p.rms);

			// Measured f0 only (no fallbacks)
			float f0_measured = 0.0f;
			if (config.use_pitch_hz && p.pitch_hz > 0.0f)
				f0_measured = p.pitch_hz;

			// Smooth pitch only when we have a valid measurement; else drop to 0
			if (f0_measured > 0.0f)
			{
				const float a = clamp01(config.pitch_smooth_alpha);
				if (state->smoothed_pitch_hz <= 0.0f)
					state->smoothed_pitch_hz = f0_measured;
				state->smoothed_pitch_hz = (1.0f - a) * state->smoothed_pitch_hz + a * f0_measured;
			}
			else
			{
				state->smoothed_pitch_hz = 0.0f;
			}

			const double f0 = (double)state->smoothed_pitch_hz;
			const double step_fund = (f0 > 0.0) ? (two_pi * std::min(f0, guard) / (double)fs) : 0.0;

			// Prosody helpers
			auto sane = [](float v, float def)
			{
				return std::isfinite(v) ? v : def;
			};
			const float flat = clamp01(sane(p.spectral_flatness, 0.0f));
			const float eratio = sane(p.spectral_energy_ratio, 1.0f);

			// Component gains (independent)
			float tone_gain = (config.enable_tone && f0 > 0.0) ? config.tone_base : 0.0f;
			if (config.use_flatness_for_tone && tone_gain > 0.0f)
				tone_gain *= (1.0f - flat);
			if (config.use_energy_ratio_for_tone && tone_gain > 0.0f)
				tone_gain *= (eratio / std::max(1e-6f, config.tone_energy_ratio_center)) * config.tone_energy_ratio_scale;

			// Partials: only if analyser provided pitch (f0>0) AND we want partials
			float part_gain = (config.enable_partials && f0 > 0.0) ? config.partials_base : 0.0f;

			// Noise
			float noise_gain = config.enable_noise ? config.noise_base : 0.0f;
			if (config.use_flatness_for_noise && noise_gain > 0.0f)
				noise_gain *= flat;
			if (config.use_energy_ratio_for_noise && noise_gain > 0.0f)
				noise_gain *= (std::max(0.0f, config.noise_energy_ratio_center - eratio) + 1.0f) * config.noise_energy_ratio_scale;

			// Clamp component gains
			const float gmin = config.min_component_gain, gmax = config.max_component_gain;
			tone_gain = std::clamp(tone_gain, gmin, gmax);
			part_gain = std::clamp(part_gain, gmin, gmax);
			noise_gain = std::clamp(noise_gain, gmin, gmax);

			// Smooth mix gains
			const float ma = clamp01(config.mix_smooth_alpha);
			state->tone_gain_z = (1.0f - ma) * state->tone_gain_z + ma * tone_gain;
			state->part_gain_z = (1.0f - ma) * state->part_gain_z + ma * part_gain;
			state->noise_gain_z = (1.0f - ma) * state->noise_gain_z + ma * noise_gain;

			tone_gain = state->tone_gain_z;
			part_gain = state->part_gain_z;
			noise_gain = state->noise_gain_z;

			// Noise LPF cutoff
			float cutoff_hz = config.noise_cutoff_default_hz;
			if (config.use_spectral_for_noise_lpf)
			{
				float base = (p.spectral_centroid_hz > 0.0f) ? p.spectral_centroid_hz : 1000.0f;
				float bw = (p.spectral_bandwidth_hz > 0.0f) ? p.spectral_bandwidth_hz : 0.0f;
				cutoff_hz = base + config.bandwidth_scale * bw;
			}
			cutoff_hz = std::clamp(cutoff_hz, 80.0f, (float)(nyq - 1.0));
			// alpha = 1 - exp(-2π * fc / fs)
			float alpha = 1.0f - std::exp(-2.0f * (float)two_pi * (cutoff_hz / (float)fs));
			alpha = std::clamp(alpha, 1e-5f, 0.9999f);

			// Sample budget
			state->sample_accum += (double)fs * (double)tick_info.delta_time;
			int count = (int)state->sample_accum;
			state->sample_accum -= count;

			if (count <= 0)
			{
				outputs.mono.samples.clear();
				state->prev_amp_linear = lin_gain;
				return;
			}

			count = std::min(count, (int)outputs.mono.samples.capacity());
			outputs.mono.samples.set_size(count);

			double local_phase[ProsodyWaveGeneratorState::MaxOsc];
			std::memcpy(local_phase, state->phase, sizeof(local_phase));
			float z1 = state->noise_z1;

			const float amp0 = state->prev_amp_linear, amp1 = lin_gain;
			const double denom = (count > 1) ? (double)(count - 1) : 1.0;

			// Cached analyser partials
			const int Np = (f0 > 0.0 && config.enable_partials) ? std::max(0, std::min(Prosody::MaxPartials, p.partial_count)) : 0;
			const bool use_abs_freq = p.partial_freq_valid;

			for (int i = 0; i < count; ++i)
			{
				const double t = (double)i / denom;
				const double amp = (double)amp0 + ((double)amp1 - (double)amp0) * t;

				double s_tone = 0.0;
				double s_part = 0.0;
				double s_noise = 0.0;

				// Fundamental (only if f0>0 and enabled)
				if (tone_gain > 0.0f && step_fund > 0.0)
				{
					s_tone = std::sin(local_phase[0]);
					local_phase[0] += step_fund;
				}

				// Partials (from analyser only; no synthetic fallback)
				if (part_gain > 0.0f && Np > 0)
				{
					for (int h = 0; h < Np; ++h)
					{
						const double gain = std::max(0.0f, p.partial_gain[h]);
						if (gain <= 0.0)
							continue;

						// If analyser supplied absolute frequencies, use them;
						// else, we require f0>0 (already checked) and assume (h+2)*f0
						const double hf = use_abs_freq ? (double)p.partial_freq_hz[h] : (double)state->smoothed_pitch_hz * (double)(h + 2);
						if (hf <= 0.0 || hf >= guard)
							continue;

						const double hstep = two_pi * hf / (double)fs;
						const int ph = 1 + h; // phase index 1..Np maps to partials
						s_part += gain * std::sin(local_phase[ph]);
						local_phase[ph] += hstep;
					}
				}

				// Noise (1-pole LPF on white)
				if (noise_gain > 0.0f)
				{
					const float white = state->noise_uniform_pm1();
					z1 = z1 + alpha * (white - z1);
					if (std::fabs(z1) < 1e-20f)
						z1 = 0.0f;
					s_noise = (double)z1;
				}

				const double s = (double)tone_gain * s_tone + (double)part_gain * s_part + (double)noise_gain * s_noise;
				outputs.mono.samples[i] = (float)(amp * s);

				// Wrap phases for fundamental + Np partials only
				const int maxPh = 1 + Np;
				for (int ph = 0; ph < std::min(maxPh, ProsodyWaveGeneratorState::MaxOsc); ++ph)
				{
					if (local_phase[ph] >= two_pi)
						local_phase[ph] -= two_pi;
					else if (local_phase[ph] < 0.0)
						local_phase[ph] += two_pi;
				}
			}

			// persist
			std::memcpy(state->phase, local_phase, sizeof(local_phase));
			state->noise_z1 = z1;
			state->prev_amp_linear = lin_gain;
		}
	};

} // namespace robotick
