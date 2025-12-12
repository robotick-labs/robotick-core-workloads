// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0
//
// ProsodyAnalyserWorkload.cpp (harmonic-driven version)

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

		float rms_smooth_alpha = 0.2f;	 // ~100 ms amplitude smoothing

		float voiced_falloff_rate_hz = 5.0f; // how quickly voiced confidence fades (1/s)

		float min_pitch_hz = 60.0f;	 // very deep adult voice
		float max_pitch_hz = 600.0f; // very high child's voice

		float harmonic_confidence_min_db = -15.0f;
		float harmonic_confidence_max_db = 25.0f;
		float harmonic_confidence_gate = 0.35f;
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

		float smoothed_rms = 0.0f;

		SpeakingRateTracker speaking_rate_state;

		RelativeVariationTracker pitch_variation_tracker;
		RelativeVariationTracker rms_variation_tracker;

		float last_jitter = 0.0f;
		float last_shimmer = 0.0f;

		bool was_voiced = false;
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
			state->smoothed_rms = apply_exponential_smoothing(state->smoothed_rms, rms, config.rms_smooth_alpha);
			prosody.rms = state->smoothed_rms;

			const float current_pitch = pitch_info.h1_f0_hz;
			const bool has_pitch = (current_pitch > 0.0f);
			prosody.is_voiced = has_pitch;
			prosody.voiced_confidence = has_pitch ? 1.0f : 0.0f;

			if (!has_pitch)
			{
				state->previous_pitch_hz = 0.0f;
				state->pitch_variation_tracker.reset();
				state->rms_variation_tracker.reset();
				state->last_jitter = 0.0f;
				state->last_shimmer = 0.0f;

				prosody = ProsodyState{};
				prosody.rms = state->smoothed_rms;
				prosody.is_voiced = false;
				prosody.voiced_confidence = 0.0f;
				prosody.is_harmonic = false;
				prosody.harmonic_confidence = 0.0f;

				decay_speaking_rate_tracker(state->speaking_rate_state, config.speaking_rate_decay);
				state->was_voiced = false;
				return;
			}

			const bool new_segment = !state->was_voiced;
			state->was_voiced = true;

			if (new_segment)
			{
				state->pitch_variation_tracker.reset();
				state->rms_variation_tracker.reset();
				state->last_jitter = 0.0f;
				state->last_shimmer = 0.0f;
				state->previous_pitch_hz = current_pitch;
			}
			prosody.pitch_hz = current_pitch;

			// --- Pitch slope (direct from the upstream pitch tracker) ---
			const float previous_pitch = state->previous_pitch_hz;
			if (!new_segment && previous_pitch > 0.0f)
			{
				prosody.pitch_slope_hz_per_s = (current_pitch - previous_pitch) / delta_time;
			}
			else
			{
				prosody.pitch_slope_hz_per_s = 0.0f;
			}
			state->previous_pitch_hz = current_pitch;

			// --- Harmonicity (HNR proxy) ---
			float harmonic_energy = 0.0f;
			for (size_t harmonic_id = 0; harmonic_id < pitch_info.harmonic_amplitudes.size(); ++harmonic_id)
			{
				const float amplitude = pitch_info.harmonic_amplitudes[harmonic_id];
				harmonic_energy += amplitude * amplitude;
			}

			prosody.harmonicity_hnr_db = compute_harmonicity_hnr_db(frame_energy, harmonic_energy, config.harmonic_floor_db);
			prosody.harmonic_confidence =
				compute_harmonic_confidence(prosody.harmonicity_hnr_db, config.harmonic_confidence_min_db, config.harmonic_confidence_max_db);
			prosody.is_harmonic = (prosody.harmonic_confidence >= config.harmonic_confidence_gate);

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

			if (!new_segment)
			{
				state->last_jitter = update_relative_variation(state->pitch_variation_tracker, current_pitch);
			}
			else
			{
				state->last_jitter = 0.0f;
			}
			prosody.jitter = state->last_jitter;

			if (!new_segment)
			{
				state->last_shimmer = update_relative_variation(state->rms_variation_tracker, rms);
			}
			else
			{
				state->last_shimmer = 0.0f;
			}
			prosody.shimmer = state->last_shimmer;

			// --- Speaking rate (EMA of voiced segment starts/sec) ---
			prosody.speaking_rate_sps = update_speaking_rate_on_voiced(state->speaking_rate_state, info.time_now, config.speaking_rate_decay);
		}
	};

} // namespace robotick
