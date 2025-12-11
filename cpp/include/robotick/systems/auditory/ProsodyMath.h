// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cmath>

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

} // namespace robotick
