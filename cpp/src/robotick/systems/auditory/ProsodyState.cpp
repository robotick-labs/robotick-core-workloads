// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

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
	ROBOTICK_STRUCT_FIELD(ProsodyState, bool, is_voiced)
	ROBOTICK_STRUCT_FIELD(ProsodyState, float, voiced_confidence)

	// ===== Temporal rhythm =====
	ROBOTICK_STRUCT_FIELD(ProsodyState, float, speaking_rate_sps)

	// ===== Pitch =====
	ROBOTICK_STRUCT_FIELD(ProsodyState, float, pitch_hz)
	ROBOTICK_STRUCT_FIELD(ProsodyState, float, pitch_slope_hz_per_s)

	// ===== Harmonic quality =====
	ROBOTICK_STRUCT_FIELD(ProsodyState, float, harmonicity_hnr_db)
	ROBOTICK_STRUCT_FIELD(ProsodyState, float, jitter)
	ROBOTICK_STRUCT_FIELD(ProsodyState, float, shimmer)

	// ===== Timbre / brightness =====
	ROBOTICK_STRUCT_FIELD(ProsodyState, float, spectral_brightness)

	// ===== Harmonics-focused descriptors =====
	ROBOTICK_STRUCT_FIELD(ProsodyState, float, h1_to_h2_db)
	ROBOTICK_STRUCT_FIELD(ProsodyState, float, harmonic_tilt_db_per_h)
	ROBOTICK_STRUCT_FIELD(ProsodyState, float, even_odd_ratio)
	ROBOTICK_STRUCT_FIELD(ProsodyState, float, harmonic_support_ratio)
	ROBOTICK_STRUCT_FIELD(ProsodyState, float, centroid_ratio)
	ROBOTICK_STRUCT_FIELD(ProsodyState, float, formant1_ratio)
	ROBOTICK_STRUCT_FIELD(ProsodyState, float, formant2_ratio)

	ROBOTICK_REGISTER_STRUCT_END(ProsodyState)

} // namespace robotick
