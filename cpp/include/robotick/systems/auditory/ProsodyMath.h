// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "robotick/framework/math/MathUtils.h"
#include "robotick/systems/auditory/HarmonicPitch.h"

#include <cmath>
#include <cstddef>

namespace robotick
{
	constexpr size_t kProsodyMaxSmoothedHarmonics = 64;

	inline float compute_harmonicity_hnr_db(const float frame_energy, const float harmonic_energy, const float floor_db)
	{
		const float safe_harmonic_energy = (harmonic_energy > 1e-12f) ? harmonic_energy : 1e-12f;
		const float residual_energy = frame_energy - safe_harmonic_energy;
		const float safe_noise_energy = (residual_energy > 1e-12f) ? residual_energy : 1e-12f;

		float harmonicity_db = 10.0f * log10f(safe_harmonic_energy / safe_noise_energy);
		return (harmonicity_db < floor_db) ? floor_db : harmonicity_db;
	}

	inline float compute_harmonic_confidence(const float hnr_db, const float min_db, const float max_db)
	{
		const float clamped_min = robotick::min(min_db, max_db - 1e-3f);
		const float clamped_max = robotick::max(max_db, clamped_min + 1e-3f);
		const float normalized = (hnr_db - clamped_min) / (clamped_max - clamped_min);
		return robotick::clamp(normalized, 0.0f, 1.0f);
	}

	struct FormantRatios
	{
		float first = 0.0f;
		float second = 0.0f;
	};

	inline FormantRatios compute_formant_ratios(const HarmonicPitchResult& hp, const float sample_rate_hz)
	{
		FormantRatios result{};

		const size_t harmonic_count = hp.harmonic_amplitudes.size();
		if (harmonic_count == 0 || hp.h1_f0_hz <= 0.0f || sample_rate_hz <= 0.0f)
		{
			return result;
		}

		float smoothed_db[kProsodyMaxSmoothedHarmonics];
		const size_t N = robotick::min(harmonic_count, kProsodyMaxSmoothedHarmonics);

		for (size_t i = 0; i < N; ++i)
		{
			const double a0 = 20.0 * log10(robotick::max(1e-12f, hp.harmonic_amplitudes[i]));
			const double aL = 20.0 * log10(robotick::max(1e-12f, hp.harmonic_amplitudes[(i > 0) ? i - 1 : i]));
			const double aR = 20.0 * log10(robotick::max(1e-12f, hp.harmonic_amplitudes[(i + 1 < N) ? i + 1 : i]));
			smoothed_db[i] = static_cast<float>((aL + a0 + aR) / 3.0);
		}

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

		const float nyquist_hz = robotick::max(1.0f, 0.5f * sample_rate_hz);
		if (best_i >= 0)
		{
			const float formant_freq = static_cast<float>(best_i + 1) * hp.h1_f0_hz;
			result.first = robotick::clamp(formant_freq / nyquist_hz, 0.0f, 1.0f);
		}
		if (second_i >= 0)
		{
			const float formant_freq = static_cast<float>(second_i + 1) * hp.h1_f0_hz;
			result.second = robotick::clamp(formant_freq / nyquist_hz, 0.0f, 1.0f);
		}

		return result;
	}

	struct RelativeVariationTracker
	{
		float previous_value = 0.0f;
		bool has_previous_value = false;

		inline void reset()
		{
			previous_value = 0.0f;
			has_previous_value = false;
		}
	};

	inline float update_relative_variation(RelativeVariationTracker& tracker, const float current_value)
	{
		if (current_value <= 0.0f)
		{
			tracker.reset();
			return 0.0f;
		}

		if (!tracker.has_previous_value)
		{
			tracker.previous_value = current_value;
			tracker.has_previous_value = true;
			return 0.0f;
		}

		const float previous_value = tracker.previous_value;
		tracker.previous_value = current_value;

		if (previous_value <= 0.0f)
		{
			return 0.0f;
		}

		const float delta = fabsf(current_value - previous_value);
		return delta / previous_value;
	}

	struct HarmonicDescriptors
	{
		float h1_to_h2_db = 0.0f;
		float harmonic_tilt_db_per_h = 0.0f;
		float even_odd_ratio = 1.0f;
		float harmonic_support_ratio = 0.0f;
		float centroid_ratio = 0.0f;
		float formant1_ratio = 0.0f;
		float formant2_ratio = 0.0f;
	};

	inline HarmonicDescriptors compute_harmonic_descriptors(const HarmonicPitchResult& hp, const float sample_rate_hz)
	{
		HarmonicDescriptors descriptors{};

		const size_t harmonic_count = hp.harmonic_amplitudes.size();
		if (harmonic_count == 0 || hp.h1_f0_hz <= 0.0f)
		{
			return descriptors;
		}

		const float h1 = hp.harmonic_amplitudes[0];
		const float h2 = (harmonic_count >= 2) ? hp.harmonic_amplitudes[1] : 1e-6f;
		const auto db = [](float x)
		{
			return 20.0f * log10f(robotick::max(1e-12f, x));
		};
		descriptors.h1_to_h2_db = db(h1) - db(h2);

		double sx = 0.0;
		double sy = 0.0;
		double sxy = 0.0;
		double sx2 = 0.0;
		double total = 0.0;
		double weighted_index_sum = 0.0;
		double even_sum = 0.0;
		double odd_sum = 0.0;
		int support_count = 0;

		const float rel_thresh = robotick::max(1e-6f, h1 * powf(10.0f, -12.0f / 20.0f));

		for (size_t i = 0; i < harmonic_count; ++i)
		{
			const double idx = static_cast<double>(i + 1);
			const double amplitude = static_cast<double>(robotick::max(1e-12f, hp.harmonic_amplitudes[i]));
			const double amplitude_db = 20.0 * log10(amplitude);

			sx += idx;
			sy += amplitude_db;
			sxy += idx * amplitude_db;
			sx2 += idx * idx;

			total += amplitude;
			weighted_index_sum += idx * amplitude;

			if (((i + 1) % 2) == 0)
			{
				even_sum += amplitude;
			}
			else
			{
				odd_sum += amplitude;
			}

			if (amplitude >= rel_thresh)
			{
				support_count++;
			}
		}

		const double n = static_cast<double>(harmonic_count);
		const double denom = robotick::max(1e-9, (n * sx2 - sx * sx));
		const double slope_db_per_h = (n * sxy - sx * sy) / denom;
		descriptors.harmonic_tilt_db_per_h = static_cast<float>(slope_db_per_h);

		descriptors.even_odd_ratio = (odd_sum > 0.0) ? static_cast<float>(even_sum / odd_sum) : 1.0f;
		descriptors.harmonic_support_ratio = static_cast<float>(support_count) / static_cast<float>(harmonic_count);
		descriptors.centroid_ratio = (total > 0.0) ? static_cast<float>((weighted_index_sum / total) / n) : 0.0f;

		const FormantRatios formant_ratios = compute_formant_ratios(hp, sample_rate_hz);
		descriptors.formant1_ratio = formant_ratios.first;
		descriptors.formant2_ratio = formant_ratios.second;

		return descriptors;
	}

	inline float compute_spectral_brightness(const HarmonicPitchResult& hp)
	{
		if (hp.h1_f0_hz <= 0.0f)
		{
			return 0.0f;
		}

		const size_t num_harmonics = hp.harmonic_amplitudes.size();
		if (num_harmonics < 2)
		{
			return 0.0f;
		}

		double sum_x = 0.0;
		double sum_y = 0.0;
		double sum_xy = 0.0;
		double sum_x2 = 0.0;

		for (size_t harmonic_id = 0; harmonic_id < num_harmonics; ++harmonic_id)
		{
			const double frequency = static_cast<double>(harmonic_id + 1) * hp.h1_f0_hz;
			const double amplitude = robotick::max(1e-12, static_cast<double>(hp.harmonic_amplitudes[harmonic_id]));
			const double log_frequency = log10(frequency);
			const double log_amplitude = log10(amplitude);

			sum_x += log_frequency;
			sum_y += log_amplitude;
			sum_xy += log_frequency * log_amplitude;
			sum_x2 += log_frequency * log_frequency;
		}

		const double n = static_cast<double>(num_harmonics);
		const double mean_x = sum_x / n;
		const double mean_y = sum_y / n;
		const double numerator = sum_xy - n * mean_x * mean_y;
		const double denominator = sum_x2 - n * mean_x * mean_x;

		if (fabs(denominator) < 1e-12)
		{
			return 0.0f;
		}

		const double slope = numerator / denominator;
		return static_cast<float>(20.0 * slope);
	}

	inline float apply_exponential_smoothing(const float previous_value, const float current_input, const float alpha)
	{
		const float clamped_alpha = robotick::clamp(alpha, 0.0f, 1.0f);
		return (1.0f - clamped_alpha) * previous_value + clamped_alpha * current_input;
	}

	inline float update_voiced_confidence(const bool voiced_now, const float current_confidence, const float delta_time, const float falloff_rate_hz)
	{
		if (voiced_now)
		{
			return 1.0f;
		}

		// Silent frame: reduce confidence linearly based on elapsed time and the configured falloff rate.
		const float decay = delta_time * falloff_rate_hz;
		return robotick::max(0.0f, current_confidence - decay);
	}

	inline float update_speaking_rate_sps(float current_tracker, float instant_rate, float decay, float silence_duration_sec)
	{
		if (silence_duration_sec <= 0.0f)
		{
			return current_tracker;
		}

		// Convert the configured decay into a clamp-safe smoothing factor for the EMA
		const float alpha = robotick::clamp(decay, 0.0f, 0.999f);
		// For long pauses, fall back to using the actual pause duration (so rate never sticks at zero).
		const float effective_rate = (silence_duration_sec > 2.0f) ? (1.0f / silence_duration_sec) : instant_rate;
		return alpha * current_tracker + (1.0f - alpha) * effective_rate;
	}

	struct SpeakingRateTracker
	{
		float tracker = 0.0f;
		float last_voiced_onset_time = 0.0f;
		bool was_voiced = false;
	};

	inline void decay_speaking_rate_tracker(SpeakingRateTracker& state, float speaking_rate_decay)
	{
		state.tracker *= speaking_rate_decay;
		state.was_voiced = false;
	}

	inline float update_speaking_rate_on_voiced(SpeakingRateTracker& state, float time_now, float speaking_rate_decay)
	{
		if (!state.was_voiced)
		{
			const float gap_seconds = robotick::max(1e-6f, time_now - state.last_voiced_onset_time);
			const float instant_rate = (gap_seconds > 0.05f) ? (1.0f / gap_seconds) : 0.0f;
			state.tracker = update_speaking_rate_sps(state.tracker, instant_rate, speaking_rate_decay, gap_seconds);
			state.last_voiced_onset_time = time_now;
		}

		state.was_voiced = true;
		return state.tracker;
	}

} // namespace robotick
