
#pragma once

#include "robotick/framework/common/FixedVector.h"
#include "robotick/systems/auditory/HarmonicPitch.h"

#include <cstdint>

namespace robotick
{
	struct SourceCandidate
	{
		float h1_f0_hz = 0.0f;					// Detected fundamental frequency (Hz)
		HarmonicAmplitudes harmonic_amplitudes; // Raw amplitudes for h1, h2, ... up to MaxHarmonics

		float get_h1_amplitude() const { return harmonic_amplitudes.size() > 0 ? harmonic_amplitudes[0] : 0.0f; }
	};

	using SourceCandidates8 = FixedVector<SourceCandidate, 8>;

} // namespace robotick
