// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "robotick/framework/math/MathUtils.h"
#include "robotick/systems/auditory/HarmonicPitch.h"

#include <cmath>
#include <cstddef>

namespace robotick
{
	inline float compute_harmonicity_hnr_db(const float frame_energy, const float harmonic_energy, const float floor_db)
	{
		const float safe_harmonic_energy = (harmonic_energy > 1e-12f) ? harmonic_energy : 1e-12f;
		const float residual_energy = frame_energy - safe_harmonic_energy;
		const float safe_noise_energy = (residual_energy > 1e-12f) ? residual_energy : 1e-12f;

		float harmonicity_db = 10.0f * log10f(safe_harmonic_energy / safe_noise_energy);
		return (harmonicity_db < floor_db) ? floor_db : harmonicity_db;
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

		float smoothed_db[64];
		const size_t N = robotick::min(harmonic_count, static_cast<size_t>(64));

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

} // namespace robotick
