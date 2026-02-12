// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0
//
// HarmonicPitch.h  (lean header: declarations only)

#pragma once

#include "robotick/framework/containers/FixedVector.h"

#include <cstdint>

namespace robotick
{
	namespace harmonic_pitch
	{
		static constexpr size_t MaxHarmonics = 16;
	};

	using HarmonicAmplitudes = FixedVector<float, harmonic_pitch::MaxHarmonics>;

	struct HarmonicPitchResult
	{
		float h1_f0_hz = 0.0f;					// Detected fundamental frequency (Hz)
		HarmonicAmplitudes harmonic_amplitudes; // Raw amplitudes for h1, h2, ... up to MaxHarmonics

		float get_h1_amplitude() const { return harmonic_amplitudes.size() > 0 ? harmonic_amplitudes[0] : 0.0f; }
	};

} // namespace robotick
