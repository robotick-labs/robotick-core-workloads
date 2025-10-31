// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#include "robotick/api.h"

#include "robotick/systems/AudioBuffer.h"
#include "robotick/systems/AudioSystem.h"

#include <SDL2/SDL.h>
#include <cmath>
#include <mutex>

namespace robotick
{
	struct SpeakerInputs
	{
		AudioBuffer64 left;
		AudioBuffer64 right;
	};

	struct SpeakerWorkload
	{
		SpeakerInputs inputs;

		void load() { AudioSystem::init(); }

		void tick(const TickInfo&)
		{
			const bool hasL = inputs.left.size() > 0;
			const bool hasR = inputs.right.size() > 0;
			if (!hasL && !hasR)
				return;

			const size_t frames =
				hasL && hasR ? std::min(inputs.left.size(), inputs.right.size()) : (hasL ? inputs.left.size() : inputs.right.size());

			std::vector<float> interleaved(frames * 2);
			for (size_t i = 0; i < frames; ++i)
			{
				const float l = hasL ? inputs.left[i] : 0.0f;
				const float r = hasR ? inputs.right[i] : 0.0f;
				interleaved[2 * i + 0] = l;
				interleaved[2 * i + 1] = r;
			}

			AudioSystem::write_interleaved_stereo(interleaved.data(), frames);
		}
	};

} // namespace robotick
