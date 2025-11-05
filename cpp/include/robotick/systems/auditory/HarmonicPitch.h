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
		float min_amplitude = 0.05f;			// minimum envelope value for it to be considered an interesting feature
		float min_peak_falloff_norm = 0.1f;		// minimum falloff from a peak (as a fraction of its (peak-value - min_amplitude))
												// 	for it to count as a peak
		float harmonic_tolerance_cents = 50.0f; // i.e. hormonic's peak must be within 50% of a semitone to count as a matching harmonic
		bool allow_single_peak_mode = true;
	};

	using HarmonicAmplitudes = FixedVector<float, harmonic_pitch::MaxHarmonics>;

	struct HarmonicPitchResult
	{
		float h1_f0_hz = 0.0f;					// Detected fundamental frequency (Hz)
		HarmonicAmplitudes harmonic_amplitudes; // Raw amplitudes for h1, h2, ... up to MaxHarmonics

		float get_h1_amplitude() const { return harmonic_amplitudes.size() > 0 ? harmonic_amplitudes[0] : 0.0f; }
	};

	class HarmonicPitch
	{
	  public:
		static bool find_harmonic_features(
			const HarmonicPitchSettings& settings, const AudioBuffer128& centers, const AudioBuffer128& envelope, HarmonicPitchResult& result);
	};

} // namespace robotick
