// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#if defined(ROBOTICK_PLATFORM_DESKTOP) || defined(ROBOTICK_PLATFORM_LINUX)

#include "robotick/api.h"
#include "robotick/systems/audio/AudioSystem.h"
#include "robotick/systems/auditory/CochlearFrame.h"
#include "robotick/systems/auditory/SnakePitchTracker.h"

namespace robotick
{
	struct HarmonicPitchConfig
	{
		SnakePitchTrackerConfig settings;
	};

	struct HarmonicPitchInputs
	{
		CochlearFrame cochlear_frame;
	};

	struct HarmonicPitchOutputs
	{
		HarmonicPitchResult pitch_info;
	};

	struct HarmonicPitchState
	{
		SnakePitchTracker tracker;
	};

	struct HarmonicPitchWorkload
	{
		HarmonicPitchConfig config;
		HarmonicPitchInputs inputs;
		HarmonicPitchOutputs outputs;

		State<HarmonicPitchState> state;

		void start(float /*tick_rate_hz*/)
		{
			state->tracker.configure(config.settings);
			state->tracker.reset();
		}

		void tick(const TickInfo&)
		{
			HarmonicPitchResult result{};
			if (state->tracker.update(inputs.cochlear_frame, result))
			{
				outputs.pitch_info = result;
			}
			else
			{
				outputs.pitch_info = HarmonicPitchResult{};
			}
		}
	};

} // namespace robotick

#endif // ROBOTICK_PLATFORM_DESKTOP || ROBOTICK_PLATFORM_LINUX
