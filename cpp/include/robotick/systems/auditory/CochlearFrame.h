#pragma once

#include "robotick/systems/audio/AudioFrame.h"

namespace robotick
{
	struct CochlearFrame
	{
		AudioBuffer128 envelope;
		AudioBuffer128 fine_phase;
		AudioBuffer128 modulation_power;
		double timestamp = 0.0;
	};

} // namespace robotick
