
#pragma once

#include "robotick/framework/common/FixedVector.h"

#include <cstdint>

namespace robotick
{
	struct SourceCandidate
	{
		float centroid_freq_hz = 0.0f;
		float harmonicity = 0.0f;		 // 0..1 sieve score (instant + temporal)
		float amplitude = 0.0f;			 // matched energy proxy
		float modulation_rate = 0.0f;	 // Hz (2..10)
		float pitch_hz = 0.0f;			 // f0 estimate
		float bandwidth_hz = 0.0f;		 // rough spectral spread
		float temporal_coherence = 0.0f; // 0..1 envelope correlation score
	};

	using SourceCandidates8 = FixedVector<SourceCandidate, 8>;

} // namespace robotick
