// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#include "robotick/systems/auditory/HarmonicPitch.h"

#include "robotick/api.h"

namespace robotick
{
	ROBOTICK_REGISTER_FIXED_VECTOR(HarmonicAmplitudes, float);

	ROBOTICK_REGISTER_STRUCT_BEGIN(HarmonicPitchResult)
	ROBOTICK_STRUCT_FIELD(HarmonicPitchResult, float, h1_f0_hz)
	ROBOTICK_STRUCT_FIELD(HarmonicPitchResult, HarmonicAmplitudes, harmonic_amplitudes)
	ROBOTICK_REGISTER_STRUCT_END(HarmonicPitchResult)

} // namespace robotick

