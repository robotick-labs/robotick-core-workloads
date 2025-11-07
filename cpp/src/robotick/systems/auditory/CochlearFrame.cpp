// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#include "robotick/systems/auditory/CochlearFrame.h"

namespace robotick
{
	ROBOTICK_REGISTER_STRUCT_BEGIN(CochlearFrame)
	ROBOTICK_STRUCT_FIELD(CochlearFrame, AudioBuffer128, envelope)
	ROBOTICK_STRUCT_FIELD(CochlearFrame, AudioBuffer128, fine_phase)
	ROBOTICK_STRUCT_FIELD(CochlearFrame, AudioBuffer128, modulation_power)
	ROBOTICK_STRUCT_FIELD(CochlearFrame, double, timestamp)
	ROBOTICK_STRUCT_FIELD(CochlearFrame, AudioBuffer128, band_center_hz)
	ROBOTICK_REGISTER_STRUCT_END(CochlearFrame)

} // namespace robotick
