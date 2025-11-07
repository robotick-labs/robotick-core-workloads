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

		static inline float db(float x) { return 20.0f * std::log10(std::max(1e-12f, x)); }

		void compute_harmonic_descriptors(const HarmonicPitchResult& hp, ProsodyState& prosody)
		{
			const size_t H = hp.harmonic_amplitudes.size();
			if (H == 0 || hp.h1_f0_hz <= 0.0f)
			{
				prosody.h1_to_h2_db = 0.0f;
				prosody.harmonic_tilt_db_per_h = 0.0f;
				prosody.even_odd_ratio = 1.0f;
				prosody.harmonic_support_ratio = 0.0f;
				prosody.centroid_ratio = 0.0f;
				prosody.formant1_ratio = 0.0f;
				prosody.formant2_ratio = 0.0f;
				return;
			}

			// H1 vs H2 (in dB). If H2 missing, treat as very low.
			const float h1 = hp.harmonic_amplitudes[0];
			const float h2 = (H >= 2) ? hp.harmonic_amplitudes[1] : 1e-6f;
			prosody.h1_to_h2_db = db(h1) - db(h2);

			// Linear fit of dB(ampl) vs harmonic index -> simple tilt per harmonic
			double sx = 0.0, sy = 0.0, sxy = 0.0, sx2 = 0.0;
			double total = 0.0, weighted_index_sum = 0.0;
			double even_sum = 0.0, odd_sum = 0.0;
			int support_count = 0;

			// Relative threshold vs H1 (e.g., 12 dB down)
			const float rel_thresh = std::max(1e-6f, h1 * std::pow(10.0f, -12.0f / 20.0f));

			for (size_t i = 0; i < H; ++i)
			{
				const double idx = static_cast<double>(i + 1); // harmonic number
				const double a = static_cast<double>(std::max(1e-12f, hp.harmonic_amplitudes[i]));
				const double adb = 20.0 * std::log10(a);

				sx += idx;
				sy += adb;
				sxy += idx * adb;
				sx2 += idx * idx;

				total += a;
				weighted_index_sum += idx * a;

				if (((i + 1) % 2) == 0)
				{
					even_sum += a;
				}
				else
				{
					odd_sum += a;
				}

				if (a >= rel_thresh)
				{
					support_count++;
				}
			}

			const double n = static_cast<double>(H);
			const double denom = std::max(1e-9, (n * sx2 - sx * sx));
			const double slope_db_per_h = (n * sxy - sx * sy) / denom; // dB per harmonic increase
			prosody.harmonic_tilt_db_per_h = static_cast<float>(slope_db_per_h);

			prosody.even_odd_ratio = (odd_sum > 0.0) ? static_cast<float>(even_sum / odd_sum) : 1.0f;
			prosody.harmonic_support_ratio = static_cast<float>(support_count) / static_cast<float>(H);
			prosody.centroid_ratio = (total > 0.0) ? static_cast<float>((weighted_index_sum / total) / n) : 0.0f;

			// Very rough “formant” peaks over the harmonic envelope: smooth + local maxima over dB
			// Simple 3-point check after a tiny moving average in index domain
			float smoothed_db[64]; // enough for your current MaxHarmonics
			const size_t N = std::min(H, static_cast<size_t>(64));

			// 3-tap moving average on dB amplitude
			for (size_t i = 0; i < N; ++i)
			{
				const double a0 = 20.0 * std::log10(std::max(1e-12f, hp.harmonic_amplitudes[i]));
				const double aL = 20.0 * std::log10(std::max(1e-12f, hp.harmonic_amplitudes[(i > 0) ? i - 1 : i]));
				const double aR = 20.0 * std::log10(std::max(1e-12f, hp.harmonic_amplitudes[(i + 1 < N) ? i + 1 : i]));
				smoothed_db[i] = static_cast<float>((aL + a0 + aR) / 3.0);
			}

			// Find top two local maxima indices (by dB)
			int best_i = -1, second_i = -1;
			float best_v = -1e9f, second_v = -1e9f;
			for (size_t i = 1; i + 1 < N; ++i)
			{
				const float v = smoothed_db[i];
				if (v > smoothed_db[i - 1] && v >= smoothed_db[i + 1])
				{
					if (v > best_v)
					{
						second_v = best_v;
						second_i = best_i;
						best_v = v;
						best_i = static_cast<int>(i);
					}
					else if (v > second_v)
					{
						second_v = v;
						second_i = static_cast<int>(i);
					}
				}
			}

			// Normalise to 0..1 by harmonic count
			const float denom_idx = (N > 1) ? static_cast<float>(N - 1) : 1.0f;
			prosody.formant1_ratio = (best_i >= 0) ? static_cast<float>(best_i) / denom_idx : 0.0f;
			prosody.formant2_ratio = (second_i >= 0) ? static_cast<float>(second_i) / denom_idx : 0.0f;
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

			// --- harmonic descriptors ---
			compute_harmonic_descriptors(pitch_info, prosody);

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
