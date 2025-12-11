// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0
//
// ProsodyAnalyserWorkload.cpp (harmonic-driven version with temporal smoothing)

#include "robotick/api.h"
#include "robotick/systems/audio/AudioFrame.h"
#include "robotick/systems/auditory/HarmonicPitch.h"
#include "robotick/systems/auditory/ProsodyMath.h"
#include "robotick/systems/auditory/ProsodyState.h"

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
		bool was_voiced = false;

		float smoothed_pitch_hz = 0.0f;
		float smoothed_rms = 0.0f;

		float speaking_rate_tracker = 0.0f;
		float last_voiced_onset_time = 0.0f;

		RelativeVariationTracker pitch_variation_tracker;
		RelativeVariationTracker rms_variation_tracker;
	};

	struct ProsodyAnalyserWorkload
	{
		ProsodyAnalyserConfig config;
		ProsodyAnalyserInputs inputs;
		ProsodyAnalyserOutputs outputs;
		State<ProsodyAnalyserState> state;

		static inline float safe_div(float numerator, float denominator, float fallback = 0.0f)
		{
			if (fabsf(denominator) > 1e-12f)
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
			const float delta_time = robotick::max(1e-6f, info.delta_time);

			// --- Compute RMS from incoming samples ---
			double energy_sum = 0.0;
			for (float sample : samples)
			{
				energy_sum += static_cast<double>(sample) * static_cast<double>(sample);
			}

			const float frame_energy = robotick::max(static_cast<float>(energy_sum), 1e-12f);
			const float rms = (samples.empty()) ? 0.0f : static_cast<float>(sqrt(energy_sum / static_cast<double>(samples.size())));

			// --- Smoothed RMS ---
			state->smoothed_rms = (1.0f - config.rms_smooth_alpha) * state->smoothed_rms + config.rms_smooth_alpha * rms;
			prosody.rms = state->smoothed_rms;

			// --- Determine voiced state ---
			const bool voiced_now = (pitch_info.h1_f0_hz >= config.min_pitch_hz && pitch_info.h1_f0_hz <= config.max_pitch_hz);
			prosody.voiced_confidence = update_voiced_confidence(voiced_now, prosody.voiced_confidence, delta_time, config.voiced_falloff_rate_hz);

			if (!voiced_now)
			{
				state->previous_pitch_hz = 0.0f;
				state->smoothed_pitch_hz = 0.0f;
				state->was_voiced = false;
				state->pitch_variation_tracker.reset();
				state->rms_variation_tracker.reset();

				prosody = ProsodyState{}; // zero the struct if you like
				prosody.rms = state->smoothed_rms;
				prosody.is_voiced = false;
				prosody.voiced_confidence = 0.0f;

				// Keep the multi-second EMA slowly fading
				state->speaking_rate_tracker *= config.speaking_rate_decay;

				return;
			}

			prosody.is_voiced = true;

			// --- Smooth voiced confidence ---
			if (voiced_now)
			{
				prosody.voiced_confidence = 1.0f;
			}
			else
			{
				const float decay = delta_time * config.voiced_falloff_rate_hz;
				prosody.voiced_confidence = robotick::max(0.0f, prosody.voiced_confidence - decay);
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

			prosody.harmonicity_hnr_db = compute_harmonicity_hnr_db(frame_energy, harmonic_energy, config.harmonic_floor_db);

			// --- Spectral brightness from slope of log(freq) vs log(amplitude) ---
			prosody.spectral_brightness = compute_spectral_brightness(pitch_info);

			// --- harmonic descriptors ---
			const HarmonicDescriptors descriptors = compute_harmonic_descriptors(pitch_info, static_cast<float>(inputs.mono.sample_rate));
			prosody.h1_to_h2_db = descriptors.h1_to_h2_db;
			prosody.harmonic_tilt_db_per_h = descriptors.harmonic_tilt_db_per_h;
			prosody.even_odd_ratio = descriptors.even_odd_ratio;
			prosody.harmonic_support_ratio = descriptors.harmonic_support_ratio;
			prosody.centroid_ratio = descriptors.centroid_ratio;
			prosody.formant1_ratio = descriptors.formant1_ratio;
			prosody.formant2_ratio = descriptors.formant2_ratio;

			// --- Jitter & shimmer (rough proxies) ---
			prosody.jitter = update_relative_variation(state->pitch_variation_tracker, current_pitch);

			prosody.shimmer = update_relative_variation(state->rms_variation_tracker, rms);

			// --- Speaking rate (EMA of voiced segment starts/sec) ---
			if (!state->was_voiced)
			{
				const float gap_seconds = info.time_now - state->last_voiced_onset_time;
				// gap_seconds captures the length of the silence leading up to this voiced onset.
				const float instant_rate = (gap_seconds > 0.05f) ? (1.0f / gap_seconds) : 0.0f;
				state->speaking_rate_tracker =
					update_speaking_rate_sps(state->speaking_rate_tracker, instant_rate, config.speaking_rate_decay, gap_seconds);
				state->last_voiced_onset_time = info.time_now;
			}

			state->was_voiced = voiced_now;
			prosody.speaking_rate_sps = state->speaking_rate_tracker;
		}
	};

} // namespace robotick
