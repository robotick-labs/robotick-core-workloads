// Copyright Robotick
// SPDX-License-Identifier: Apache-2.0
//
// ProsodyWaveGeneratorWorkload.cpp (harmonic-driven version)

#include "robotick/api.h"
#include "robotick/systems/audio/AudioFrame.h"
#include "robotick/systems/audio/AudioSystem.h"
#include "robotick/systems/auditory/ProsodyState.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace robotick
{
	struct ProsodyWaveGeneratorConfig
	{
		// --- Global output ---
		float amplitude_gain_db = 0.0f;
		bool use_rms_for_amplitude = true;
		bool use_voiced_gate = true;

		// --- Fundamental tone ---
		bool enable_tone = true;
		float tone_base = 1.0f;

		// --- Synthetic partials ---
		bool enable_partials = true;
		float partials_base = 1.0f;
		int max_num_partials = 16; // synthetic harmonics beyond f0

		// --- Noise ---
		bool enable_noise = true;
		float noise_base = 0.5f;

		// --- Brightness → noise and partial shaping ---
		float brightness_to_noise_scale = 0.8f;
		float brightness_to_partial_scale = 5.0f;

		// --- Harmonicity influence ---
		float harmonicity_to_noise_scale = 0.03f;
		float harmonicity_to_partial_scale = 0.03f;

		// --- Noise coloration ---
		bool use_brightness_for_noise_lpf = true;
		float noise_cutoff_default_hz = 2000.0f;

		// --- Smoothing ---
		float mix_smooth_alpha = 0.8f;

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
		static constexpr int MaxOsc = 1 + 8; // 1 fundamental + up to 8 synthetic partials

		double sample_accum = 0.0;
		double last_step_fundamental = 0.0;
		double phase[MaxOsc] = {0.0};

		float noise_filter_state = 0.0f;

		float previous_amplitude_linear = 0.0f;
		float tone_gain_smooth = 0.0f;
		float partial_gain_smooth = 0.0f;
		float noise_gain_smooth = 0.0f;

		uint32_t random_state = 0x12345678u;
		inline float random_uniform_pm1()
		{
			uint32_t x = random_state;
			x ^= x << 13;
			x ^= x >> 17;
			x ^= x << 5;
			random_state = x;
			return static_cast<float>(static_cast<int32_t>(x) / 2147483648.0f);
		}
	};

	struct ProsodyWaveGeneratorWorkload
	{
		ProsodyWaveGeneratorConfig config;
		ProsodyWaveGeneratorInputs inputs;
		ProsodyWaveGeneratorOutputs outputs;
		State<ProsodyWaveGeneratorState> state;

		static inline float clamp01(float value)
		{
			if (value < 0.0f)
			{
				return 0.0f;
			}
			if (value > 1.0f)
			{
				return 1.0f;
			}
			return value;
		}

		void load()
		{
			AudioSystem::init();
			std::fill(std::begin(state->phase), std::end(state->phase), 0.0);
			state->noise_filter_state = 0.0f;
			state->previous_amplitude_linear = 0.0f;
			state->tone_gain_smooth = state->partial_gain_smooth = state->noise_gain_smooth = 0.0f;
			state->sample_accum = 0.0;
		}

		void start(float) { outputs.mono.sample_rate = AudioSystem::get_sample_rate(); }

		int emit_smooth_zero(AudioFrame& out, ProsodyWaveGeneratorState& s, int max_tail_samples)
		{
			const int capacity = static_cast<int>(out.samples.capacity());
			if (capacity <= 0)
			{
				return 0;
			}

			out.samples.set_size(capacity);

			const double current_value =
				std::sin(s.phase[0]) * static_cast<double>(s.tone_gain_smooth) * static_cast<double>(s.previous_amplitude_linear);
			const double slope = std::cos(s.phase[0]) * s.last_step_fundamental * static_cast<double>(s.tone_gain_smooth) *
								 static_cast<double>(s.previous_amplitude_linear);

			const int upper = std::min(max_tail_samples, capacity);
			if (upper <= 0)
			{
				return 0;
			}

			int num_samples = 0;
			if (std::fabs(slope) > 1e-9)
			{
				num_samples = static_cast<int>(std::ceil(std::fabs(current_value / slope)));
			}

			num_samples = std::clamp(num_samples, 0, upper);
			if (upper >= 4)
			{
				num_samples = std::max(num_samples, 4);
			}
			else
			{
				num_samples = upper;
			}

			double value = current_value;
			for (int sample_index = 0; sample_index < num_samples; ++sample_index)
			{
				out.samples[sample_index] = static_cast<float>(value);
				value -= slope;
			}

			for (int sample_index = num_samples; sample_index < capacity; ++sample_index)
			{
				out.samples[sample_index] = 0.0f;
			}

			s.previous_amplitude_linear = 0.0f;
			return capacity;
		}

		double compute_partial_weight(const ProsodyState& prosody, int harmonic_index_zero_based, int max_harmonics)
		{
			const int h = harmonic_index_zero_based + 1; // 1..N
			const double N = static_cast<double>(std::max(1, max_harmonics));

			// Tilt (convert dB/har to linear)
			const double tilt_db_per_h = prosody.harmonic_tilt_db_per_h;
			const double tilt_linear = std::pow(10.0, (tilt_db_per_h * (h - 1)) / 20.0);

			// Even/odd emphasis
			const float even_odd_ratio = (prosody.even_odd_ratio > 0.0f) ? prosody.even_odd_ratio : 1.0f;
			const bool is_even = ((h % 2) == 0);
			const double clamped_eo = std::clamp(static_cast<double>(even_odd_ratio), 0.25, 4.0);
			const double eo = is_even ? clamped_eo : (1.0 / clamped_eo);

			// Centroid pull
			const double centroid_ratio = std::clamp(static_cast<double>(prosody.centroid_ratio), 0.0, 1.0);
			const double center = 1.0 + centroid_ratio * (N - 1.0); // 1..N
			const double dist = std::abs(static_cast<double>(h) - center);
			const double centroid_weight = 1.0 / (1.0 + 0.15 * dist);

			// Formant bumps
			auto gaussian = [](double x, double m, double s)
			{
				const double d = (x - m) / std::max(1e-6, s);
				return std::exp(-0.5 * d * d);
			};

			const double f1 = 1.0 + std::clamp(static_cast<double>(prosody.formant1_ratio), 0.0, 1.0) * (N - 1.0);
			const double f2 = 1.0 + std::clamp(static_cast<double>(prosody.formant2_ratio), 0.0, 1.0) * (N - 1.0);
			const double formant_emphasis = 0.6 * gaussian(h, f1, 1.2) + 0.4 * gaussian(h, f2, 1.8) + 0.3; // floor

			// Support gating
			const double support_ratio = std::clamp(static_cast<double>(prosody.harmonic_support_ratio), 0.0, 1.0);
			const double support_falloff = 1.0 / (1.0 + (1.0 - support_ratio) * 0.5 * (h - 1));

			// Final weight
			double w = tilt_linear * eo * centroid_weight * formant_emphasis * support_falloff;
			return std::clamp(w, 0.0, 4.0);
		}

		void tick(const TickInfo& tick_info)
		{
			static constexpr double ns_to_sec = 1e-9;
			outputs.mono.timestamp = ns_to_sec * static_cast<double>(tick_info.time_now_ns);

			const ProsodyState& prosody = inputs.prosody_state;

			if (config.use_voiced_gate && !prosody.is_voiced)
			{
				emit_smooth_zero(outputs.mono, state.get(), 64);
				return;
			}

			const int sample_rate = outputs.mono.sample_rate;
			const double nyquist_hz = 0.5 * static_cast<double>(sample_rate);
			const double frequency_guard = 0.98 * nyquist_hz;
			const double two_pi = 6.28318530717958647692;

			// --- Global amplitude ---
			float amplitude_linear = std::pow(10.0f, config.amplitude_gain_db / 20.0f);
			if (config.use_rms_for_amplitude)
			{
				amplitude_linear *= std::max(0.0f, prosody.rms);
			}

			// --- Fundamental frequency ---
			const double f0 = (prosody.pitch_hz > 0.0f) ? prosody.pitch_hz : 0.0;
			const double step_fundamental = (f0 > 0.0) ? (two_pi * std::min(f0, frequency_guard) / static_cast<double>(sample_rate)) : 0.0;

			if (step_fundamental > 0.0)
			{
				state->last_step_fundamental = step_fundamental;
			}

			// --- Interpret expressive cues ---
			const float brightness01 = clamp01(prosody.spectral_brightness / 150.0f); // normalized heuristic
			const float harmonicity = prosody.harmonicity_hnr_db;

			// harmonic descriptors
			const float support_ratio = clamp01(prosody.harmonic_support_ratio); // 0..1: how many harmonics present

			// --- Compute component gains ---
			float tone_gain = config.enable_tone ? config.tone_base : 0.0f;
			float partials_gain = config.enable_partials ? config.partials_base : 0.0f;
			float noise_gain = config.enable_noise ? config.noise_base : 0.0f;

			// Harmonicity tends to reduce noise and increase partials
			if (config.enable_noise)
			{
				noise_gain *= (1.0f + config.brightness_to_noise_scale * brightness01);
				noise_gain *= (1.0f - config.harmonicity_to_noise_scale * std::max(0.0f, harmonicity));
				noise_gain *= (0.7f + 0.6f * (1.0f - support_ratio));
			}

			if (config.enable_partials)
			{
				partials_gain *= (1.0f + config.brightness_to_partial_scale * brightness01);
				partials_gain *= (1.0f + config.harmonicity_to_partial_scale * std::max(0.0f, harmonicity));
				partials_gain *= (0.5f + 0.5f * support_ratio);
			}

			// Clamp
			tone_gain = std::clamp(tone_gain, config.min_component_gain, config.max_component_gain);
			partials_gain = std::clamp(partials_gain, config.min_component_gain, config.max_component_gain);
			noise_gain = std::clamp(noise_gain, config.min_component_gain, config.max_component_gain);

			// Smooth gains
			const float mix_alpha = clamp01(config.mix_smooth_alpha);
			state->tone_gain_smooth = (1.0f - mix_alpha) * state->tone_gain_smooth + mix_alpha * tone_gain;
			state->partial_gain_smooth = (1.0f - mix_alpha) * state->partial_gain_smooth + mix_alpha * partials_gain;
			state->noise_gain_smooth = (1.0f - mix_alpha) * state->noise_gain_smooth + mix_alpha * noise_gain;

			tone_gain = state->tone_gain_smooth;
			partials_gain = state->partial_gain_smooth;
			noise_gain = state->noise_gain_smooth;

			// --- Noise LPF cutoff ---
			float cutoff_hz = config.noise_cutoff_default_hz;
			if (config.use_brightness_for_noise_lpf)
			{
				cutoff_hz = 400.0f + brightness01 * 3000.0f;
			}
			cutoff_hz = std::clamp(cutoff_hz, 80.0f, static_cast<float>(nyquist_hz - 1.0));
			const float alpha =
				std::clamp(1.0f - std::exp(-2.0f * static_cast<float>(two_pi) * (cutoff_hz / static_cast<float>(sample_rate))), 1e-5f, 0.9999f);

			// --- Determine how many samples to produce ---
			state->sample_accum += static_cast<double>(sample_rate) * static_cast<double>(tick_info.delta_time);
			int num_samples = static_cast<int>(state->sample_accum);
			state->sample_accum -= num_samples;

			if (num_samples <= 0)
			{
				emit_smooth_zero(outputs.mono, state.get(), 16);
				return;
			}

			num_samples = std::min(num_samples, static_cast<int>(outputs.mono.samples.capacity()));
			outputs.mono.samples.set_size(num_samples);

			double phase_local[ProsodyWaveGeneratorState::MaxOsc];
			std::memcpy(phase_local, state->phase, sizeof(phase_local));
			float noise_state = state->noise_filter_state;

			const float amplitude_start = state->previous_amplitude_linear;
			const float amplitude_end = amplitude_linear;
			const double denominator = (num_samples > 1) ? static_cast<double>(num_samples - 1) : 1.0;

			for (int sample_index = 0; sample_index < num_samples; ++sample_index)
			{
				const double t = static_cast<double>(sample_index) / denominator;
				const double amplitude =
					static_cast<double>(amplitude_start) + (static_cast<double>(amplitude_end) - static_cast<double>(amplitude_start)) * t;

				double signal_tone = 0.0;
				double signal_partials = 0.0;
				double signal_noise = 0.0;

				// --- Tone ---
				if (tone_gain > 0.0f && step_fundamental > 0.0)
				{
					signal_tone = std::sin(phase_local[0]);
					phase_local[0] += step_fundamental;
				}

				// --- Synthetic partials ---
				if (partials_gain > 0.0f && f0 > 0.0)
				{
#if ENABLE_PARTIALS_LOG
					const bool emit_log = (tick_info.tick_count % 10) == 0;
					FixedString<512> harmonic_log = "partials: ";
#endif // #if ENABLE_PARTIALS_LOG

					const int num_partials = std::clamp(config.max_num_partials, 0, ProsodyWaveGeneratorState::MaxOsc - 1);
					for (int harmonic_index = 0; harmonic_index < num_partials; ++harmonic_index)
					{
						const double harmonic_frequency = (harmonic_index + 2) * f0;
						if (harmonic_frequency >= frequency_guard)
						{
							continue;
						}

						const int phase_index = 1 + harmonic_index;

						const double baseRolloff = 1.0 / (1.0 + harmonic_index); // keep your gentle rolloff
						const double w = compute_partial_weight(
							prosody, harmonic_index, std::clamp(config.max_num_partials, 0, ProsodyWaveGeneratorState::MaxOsc - 1));
						const double harmonic_amplitude = w * baseRolloff;
						signal_partials += harmonic_amplitude * std::sin(phase_local[phase_index]);

						const double harmonic_step = two_pi * harmonic_frequency / static_cast<double>(sample_rate);
						phase_local[phase_index] += harmonic_step;

#if ENABLE_PARTIALS_LOG
						// Append to single-line log
						if (emit_log)
						{
							harmonic_log.appendf("h%d=%.3f ", harmonic_index + 1, harmonic_amplitude);
						}
#endif // #if ENABLE_PARTIALS_LOG
					}

#if ENABLE_PARTIALS_LOG
					if (emit_log)
					{
						ROBOTICK_INFO("%s", harmonic_log.c_str());
					}
#endif // #if ENABLE_PARTIALS_LOG
				}

				// --- Noise (one-pole LPF) ---
				if (noise_gain > 0.0f)
				{
					const float white_noise = state->random_uniform_pm1();
					noise_state = noise_state + alpha * (white_noise - noise_state);
					signal_noise = static_cast<double>(noise_state);
				}

				const double mixed_signal = static_cast<double>(tone_gain) * signal_tone + static_cast<double>(partials_gain) * signal_partials +
											static_cast<double>(noise_gain) * signal_noise;

				outputs.mono.samples[sample_index] = static_cast<float>(amplitude * mixed_signal);

				// Wrap phases for tone + partials
				const int max_phase_index = 1 + config.max_num_partials;
				for (int phase_index = 0; phase_index < std::min(max_phase_index, ProsodyWaveGeneratorState::MaxOsc); ++phase_index)
				{
					if (phase_local[phase_index] >= two_pi)
					{
						phase_local[phase_index] -= two_pi;
					}
					else if (phase_local[phase_index] < 0.0)
					{
						phase_local[phase_index] += two_pi;
					}
				}
			}

			// --- Persist state ---
			std::memcpy(state->phase, phase_local, sizeof(phase_local));
			state->noise_filter_state = noise_state;
			state->previous_amplitude_linear = amplitude_linear;
		}
	};

} // namespace robotick
