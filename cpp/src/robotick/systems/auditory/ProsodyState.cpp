
#include "robotick/systems/auditory/ProsodyState.h"

#include "robotick/api.h"

namespace robotick
{
	ROBOTICK_REGISTER_STRUCT_BEGIN(ProsodyState)
	// Core
	ROBOTICK_STRUCT_FIELD(ProsodyState, float, rms)
	ROBOTICK_STRUCT_FIELD(ProsodyState, float, zcr)
	ROBOTICK_STRUCT_FIELD(ProsodyState, float, pitch_hz)
	ROBOTICK_STRUCT_FIELD(ProsodyState, float, pitch_slope_hz_per_s)
	ROBOTICK_STRUCT_FIELD(ProsodyState, bool, voiced)

	// Spectral
	ROBOTICK_STRUCT_FIELD(ProsodyState, float, spectral_energy_rms)
	ROBOTICK_STRUCT_FIELD(ProsodyState, float, spectral_energy_ratio)
	ROBOTICK_STRUCT_FIELD(ProsodyState, float, spectral_centroid_hz)
	ROBOTICK_STRUCT_FIELD(ProsodyState, float, spectral_bandwidth_hz)
	ROBOTICK_STRUCT_FIELD(ProsodyState, float, spectral_flatness)

	// Advanced (stubs for now)
	ROBOTICK_STRUCT_FIELD(ProsodyState, float, speaking_rate_sps)
	ROBOTICK_STRUCT_FIELD(ProsodyState, float, jitter)
	ROBOTICK_STRUCT_FIELD(ProsodyState, float, shimmer)
	ROBOTICK_STRUCT_FIELD(ProsodyState, float, harmonicity_hnr)
	ROBOTICK_STRUCT_FIELD(ProsodyState, float, formant_f1_hz)
	ROBOTICK_STRUCT_FIELD(ProsodyState, float, formant_f2_hz)
	ROBOTICK_STRUCT_FIELD(ProsodyState, float, formant_f3_hz)
	ROBOTICK_REGISTER_STRUCT_END(ProsodyState)

} // namespace robotick