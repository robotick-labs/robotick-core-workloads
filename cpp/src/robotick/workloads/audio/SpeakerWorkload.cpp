// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#if defined(ROBOTICK_PLATFORM_DESKTOP) || defined(ROBOTICK_PLATFORM_LINUX)

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

	struct SpeakerOutputs
	{
		AudioBackpressureStats queue_stats{};
		FixedString32 last_queue_status;
	};

	struct SpeakerWorkload
	{
		SpeakerInputs inputs;
		SpeakerOutputs outputs;

		void load() { AudioSystem::init(); }

		void tick(const TickInfo&)
		{
			const bool hasL = inputs.left.samples.size() > 0;
			const bool hasR = inputs.right.samples.size() > 0;
			AudioQueueResult queue_result = AudioQueueResult::Success;
			bool issued_audio = false;

			if (hasL && hasR)
			{
				ROBOTICK_ASSERT(inputs.left.samples.size() == inputs.right.samples.size());

				ROBOTICK_ASSERT(inputs.left.sample_rate == inputs.right.sample_rate);
				ROBOTICK_ASSERT(inputs.left.sample_rate == AudioSystem::get_sample_rate());

				queue_result = AudioSystem::write_stereo(inputs.left.samples.data(), inputs.right.samples.data(), inputs.left.samples.size());
				issued_audio = true;
			}
			else if (hasL)
			{
				ROBOTICK_ASSERT(inputs.left.sample_rate == AudioSystem::get_sample_rate());
				queue_result = AudioSystem::write_mono_to_channel(0, inputs.left.samples.data(), inputs.left.samples.size());
				issued_audio = true;
			}
			else if (hasR)
			{
				ROBOTICK_ASSERT(inputs.right.sample_rate == AudioSystem::get_sample_rate());
				queue_result = AudioSystem::write_mono_to_channel(1, inputs.right.samples.data(), inputs.right.samples.size());
				issued_audio = true;
			}

			if (issued_audio)
			{
				outputs.queue_stats = AudioSystem::get_backpressure_stats();
				switch (queue_result)
				{
				case AudioQueueResult::Success:
					outputs.last_queue_status = "success";
					break;
				case AudioQueueResult::Dropped:
					outputs.last_queue_status = "dropped";
					break;
				case AudioQueueResult::Error:
				default:
					outputs.last_queue_status = "error";
					break;
				}
			}
		}
	};

} // namespace robotick

#endif // ROBOTICK_PLATFORM_DESKTOP || ROBOTICK_PLATFORM_LINUX
