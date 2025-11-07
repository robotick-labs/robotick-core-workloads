// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#include "robotick/api.h"

#include "robotick/systems/audio/AudioFrame.h"
#include "robotick/systems/audio/AudioSystem.h"

#include <SDL2/SDL.h>
#include <cmath>
#include <mutex>

namespace robotick
{
	struct SpeakerInputs
	{
		AudioFrame left;
		AudioFrame right;
	};

	struct SpeakerWorkload
	{
		SpeakerInputs inputs;

		void load() { AudioSystem::init(); }

		void tick(const TickInfo&)
		{
			const bool hasL = inputs.left.samples.size() > 0;
			const bool hasR = inputs.right.samples.size() > 0;
			if (hasL && hasR)
			{
				ROBOTICK_ASSERT(inputs.left.samples.size() == inputs.right.samples.size());

				ROBOTICK_ASSERT(inputs.left.sample_rate == inputs.right.sample_rate);
				ROBOTICK_ASSERT(inputs.left.sample_rate == AudioSystem::get_sample_rate());

				AudioSystem::write_stereo(inputs.left.samples.data(), inputs.right.samples.data(), inputs.left.samples.size());
			}
			else if (hasL)
			{
				ROBOTICK_ASSERT(inputs.left.sample_rate == AudioSystem::get_sample_rate());
				AudioSystem::write_mono_to_channel(0, inputs.left.samples.data(), inputs.left.samples.size());
			}
			else if (hasR)
			{
				ROBOTICK_ASSERT(inputs.right.sample_rate == AudioSystem::get_sample_rate());
				AudioSystem::write_mono_to_channel(1, inputs.right.samples.data(), inputs.right.samples.size());
			}
		}
	};

} // namespace robotick
