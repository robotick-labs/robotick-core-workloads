// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0
//
// HarmonicPitch.h  (lean header: declarations only)

#pragma once

#include "robotick/api.h"
#include "robotick/framework/common/FixedVector.h"
#include "robotick/systems/audio/AudioFrame.h"

#include <cstdint>

namespace robotick
{
	namespace harmonic_pitch
	{
		static constexpr size_t MaxHarmonics = 16;
	};

	struct HarmonicPitchSettings
	{
		// Selection / gating
		float min_amplitude = 0.3f;			 // minimum envelope value for it to be considered an interesting feature
		float min_peak_falloff_norm = 0.25f; // minimum falloff from a peak (as a fraction of its (peak-value - min_amplitude))
											 // 	for it to count as a peak
	};

	struct HarmonicPitchResult
	{
		float h1_f0_hz = 0.0f;
		float h1_amplitude = 0.0f;

		FixedVector<float, harmonic_pitch::MaxHarmonics> harmonic_amplitudes;
	};

	class HarmonicPitch
	{
	  public:
		static int find_strongest_f0_band_id(
			const HarmonicPitchSettings& settings, const AudioBuffer128& centers, const AudioBuffer128& envelope, HarmonicPitchResult& result);
	};

} // namespace robotick
