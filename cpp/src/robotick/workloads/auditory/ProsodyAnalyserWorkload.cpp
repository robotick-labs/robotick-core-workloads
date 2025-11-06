// Copyright Robotick
// SPDX-License-Identifier: Apache-2.0
//
// ProsodyAnalyserWorkload.cpp (harmonic-driven version with temporal smoothing)

#include "robotick/api.h"
#include "robotick/systems/audio/AudioFrame.h"
#include "robotick/systems/auditory/HarmonicPitch.h"
#include "robotick/systems/auditory/ProsodyState.h"

#include <algorithm>
#include <cmath>

namespace robotick
{
	struct ProsodyAnalyserConfig
	{
		float harmonic_floor_db = -60.0f;  // HNR clamp
		float speaking_rate_decay = 0.95f; // slower EMA smoothing for multi-second trend

		float pitch_smooth_alpha = 0.2f; // ~5-frame smoothing (~100 ms)
		float rms_smooth_alpha = 0.2f;	 // ~100 ms amplitude smoothing

		float voiced_falloff_rate_hz = 5.0f; // how quickly voiced confidence fades (1/s)

		float min_pitch_hz = 60.0f;	 // very deep adult voice
		float max_pitch_hz = 600.0f; // very high child's voice
	};

	struct ProsodyAnalyserInputs
	{
		AudioFrame mono; // for RMS
		HarmonicPitchResult pitch_info;
	};

	struct ProsodyAnalyserOutputs
	{
		ProsodyState prosody_state;
	};

	struct ProsodyAnalyserState
	{
		float previous_pitch_hz = 0.0f;
		float previous_rms = 0.0f;
		bool was_voiced = false;

		float smoothed_pitch_hz = 0.0f;
		float smoothed_rms = 0.0f;

		float speaking_rate_tracker = 0.0f;
		float last_voiced_onset_time = 0.0f;
	};

	struct ProsodyAnalyserWorkload
	{
		ProsodyAnalyserConfig config;
		ProsodyAnalyserInputs inputs;
		ProsodyAnalyserOutputs outputs;
		State<ProsodyAnalyserState> state;

		static inline float safe_div(float numerator, float denominator, float fallback = 0.0f)
		{
			if (std::fabs(denominator) > 1e-12f)
			{
				return numerator / denominator;
			}
			return fallback;
		}

		// ----------------------------------------------------------
		// Main tick: compute expressive prosody from harmonics
		// ----------------------------------------------------------
		void tick(const TickInfo& info)
		{
			auto& prosody = outputs.prosody_state;
			const auto& pitch_info = inputs.pitch_info;
			const auto& samples = inputs.mono.samples;
			const float delta_time = std::max(1e-6f, info.delta_time);

			// --- Compute RMS from incoming samples ---
			double energy_sum = 0.0;
			for (float sample : samples)
			{
				energy_sum += static_cast<double>(sample) * static_cast<double>(sample);
			}

			const float rms = (samples.empty()) ? 0.0f : static_cast<float>(std::sqrt(energy_sum / static_cast<double>(samples.size())));

			// --- Smoothed RMS ---
			state->smoothed_rms = (1.0f - config.rms_smooth_alpha) * state->smoothed_rms + config.rms_smooth_alpha * rms;
			prosody.rms = state->smoothed_rms;

			// --- Determine voiced state ---
			const bool voiced_now = (pitch_info.h1_f0_hz >= config.min_pitch_hz && pitch_info.h1_f0_hz <= config.max_pitch_hz);
			if (!voiced_now)
			{
				state->previous_pitch_hz = 0.0f;
				state->smoothed_pitch_hz = 0.0f;
				state->was_voiced = false;

				prosody = ProsodyState{}; // zero the struct if you like
				prosody.rms = state->smoothed_rms;
				prosody.voiced = false;
				prosody.voiced_confidence = 0.0f;

				// Keep the multi-second EMA slowly fading
				state->speaking_rate_tracker *= config.speaking_rate_decay;

				return;
			}

			prosody.voiced = true;

			// --- Smooth voiced confidence ---
			if (voiced_now)
			{
				prosody.voiced_confidence = 1.0f;
			}
			else
			{
				const float decay = delta_time * config.voiced_falloff_rate_hz;
				prosody.voiced_confidence = std::max(0.0f, prosody.voiced_confidence - decay);
			}

			// --- Pitch smoothing ---
			const float current_pitch = pitch_info.h1_f0_hz;
			state->smoothed_pitch_hz = (1.0f - config.pitch_smooth_alpha) * state->smoothed_pitch_hz + config.pitch_smooth_alpha * current_pitch;
			prosody.pitch_hz = state->smoothed_pitch_hz;

			// --- Pitch slope (use smoothed pitch) ---
			const float previous_pitch = state->previous_pitch_hz;
			if (previous_pitch > 0.0f && state->smoothed_pitch_hz > 0.0f)
			{
				prosody.pitch_slope_hz_per_s = (state->smoothed_pitch_hz - previous_pitch) / delta_time;
			}
			else
			{
				prosody.pitch_slope_hz_per_s = 0.0f;
			}

			state->previous_pitch_hz = state->smoothed_pitch_hz;

			// --- Harmonicity (HNR proxy) ---
			float harmonic_energy = 0.0f;
			for (size_t harmonic_id = 0; harmonic_id < pitch_info.harmonic_amplitudes.size(); ++harmonic_id)
			{
				const float amplitude = pitch_info.harmonic_amplitudes[harmonic_id];
				harmonic_energy += amplitude * amplitude;
			}

			const float total_energy = std::max(harmonic_energy, 1e-12f);
			const float noise_energy = std::max(1e-12f, total_energy - harmonic_energy);
			float harmonicity_db = 10.0f * std::log10(harmonic_energy / noise_energy);
			if (harmonicity_db < config.harmonic_floor_db)
			{
				harmonicity_db = config.harmonic_floor_db;
			}
			prosody.harmonicity_hnr_db = harmonicity_db;

			// --- Spectral brightness from slope of log(freq) vs log(amplitude) ---
			if (pitch_info.harmonic_amplitudes.size() >= 2)
			{
				double sum_x = 0.0;
				double sum_y = 0.0;
				double sum_xy = 0.0;
				double sum_x2 = 0.0;
				const size_t num_harmonics = pitch_info.harmonic_amplitudes.size();

				for (size_t harmonic_id = 0; harmonic_id < num_harmonics; ++harmonic_id)
				{
					const double frequency = (harmonic_id + 1) * pitch_info.h1_f0_hz;
					const double amplitude = std::max(1e-12, static_cast<double>(pitch_info.harmonic_amplitudes[harmonic_id]));
					const double log_frequency = std::log10(frequency);
					const double log_amplitude = std::log10(amplitude);

					sum_x += log_frequency;
					sum_y += log_amplitude;
					sum_xy += log_frequency * log_amplitude;
					sum_x2 += log_frequency * log_frequency;
				}

				const double mean_x = sum_x / static_cast<double>(num_harmonics);
				const double mean_y = sum_y / static_cast<double>(num_harmonics);
				const double numerator = sum_xy - static_cast<double>(num_harmonics) * mean_x * mean_y;
				const double denominator = sum_x2 - static_cast<double>(num_harmonics) * mean_x * mean_x;
				const double slope = safe_div(numerator, denominator, 0.0);

				prosody.spectral_brightness = static_cast<float>(-20.0 * slope);
			}
			else
			{
				prosody.spectral_brightness = 0.0f;
			}

			// --- Jitter & shimmer (rough proxies) ---
			const float pitch_delta = std::fabs(current_pitch - previous_pitch);
			prosody.jitter = (previous_pitch > 0.0f) ? (pitch_delta / previous_pitch) : 0.0f;

			const float rms_delta = std::fabs(state->smoothed_rms - state->previous_rms);
			prosody.shimmer = (state->previous_rms > 0.0f) ? (rms_delta / state->previous_rms) : 0.0f;

			state->previous_rms = state->smoothed_rms;

			// --- Speaking rate (EMA of voiced segment starts/sec) ---
			if (!state->was_voiced)
			{
				const float gap_seconds = info.time_now - state->last_voiced_onset_time;
				if (gap_seconds > 0.05f && gap_seconds < 2.0f)
				{
					const float instant_rate = 1.0f / gap_seconds;
					state->speaking_rate_tracker =
						config.speaking_rate_decay * state->speaking_rate_tracker + (1.0f - config.speaking_rate_decay) * instant_rate;
				}
				state->last_voiced_onset_time = info.time_now;
			}

			state->was_voiced = voiced_now;
			prosody.speaking_rate_sps = state->speaking_rate_tracker;
		}
	};

} // namespace robotick
