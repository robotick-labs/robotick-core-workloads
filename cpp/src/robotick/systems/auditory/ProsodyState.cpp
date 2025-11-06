
#include "robotick/systems/auditory/ProsodyState.h"

#include "robotick/api.h"

#include <cstdlib>
#include <cstring>

namespace robotick
{
	// ProsodyState Registration: ===========================

	ROBOTICK_REGISTER_STRUCT_BEGIN(ProsodyState)

	// ===== Core =====
	ROBOTICK_STRUCT_FIELD(ProsodyState, float, rms)
	ROBOTICK_STRUCT_FIELD(ProsodyState, bool, voiced)
	ROBOTICK_STRUCT_FIELD(ProsodyState, float, voiced_confidence)

	// ===== Pitch =====
	ROBOTICK_STRUCT_FIELD(ProsodyState, float, pitch_hz)
	ROBOTICK_STRUCT_FIELD(ProsodyState, float, pitch_slope_hz_per_s)

	// ===== Harmonic quality =====
	ROBOTICK_STRUCT_FIELD(ProsodyState, float, harmonicity_hnr_db)
	ROBOTICK_STRUCT_FIELD(ProsodyState, float, jitter)
	ROBOTICK_STRUCT_FIELD(ProsodyState, float, shimmer)

	// ===== Timbre / brightness =====
	ROBOTICK_STRUCT_FIELD(ProsodyState, float, spectral_brightness)

	// ===== Temporal rhythm =====
	ROBOTICK_STRUCT_FIELD(ProsodyState, float, speaking_rate_sps)

	ROBOTICK_REGISTER_STRUCT_END(ProsodyState)

} // namespace robotick