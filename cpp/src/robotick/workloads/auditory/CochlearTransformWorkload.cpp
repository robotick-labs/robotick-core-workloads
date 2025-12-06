// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0
//
// CochlearTransformWorkload.cpp  (thin wrapper around robotick::CochlearTransform)

#if defined(ROBOTICK_PLATFORM_DESKTOP) || defined(ROBOTICK_PLATFORM_LINUX)

#include "robotick/api.h"
#include "robotick/systems/audio/AudioSystem.h"
#include "robotick/systems/auditory/CochlearTransform.h"

namespace robotick
{
	struct CochlearTransformInputs
	{
		AudioFrame mono;
	};

	struct CochlearTransformOutputs
	{
		CochlearFrame cochlear_frame;
	};

	struct CochlearTransformWorkload
	{
		CochlearTransformConfig config;
		CochlearTransformInputs inputs;
		CochlearTransformOutputs outputs;
		StatePtr<CochlearTransformState> state;

		void load()
		{
			AudioSystem::init();
			const uint32_t input_rate = AudioSystem::get_input_sample_rate();
			state->sample_rate = (input_rate != 0) ? input_rate : AudioSystem::get_sample_rate();

			// Derived rate for envelope/filter math.
			state->frame_rate_hz = static_cast<double>(state->sample_rate) / static_cast<double>(CochlearTransformState::hop_size);

			// Respect AudioBuffer128 capacity.
			config.num_bands = robotick::min(config.num_bands, static_cast<uint16_t>(AudioBuffer128::capacity()));

			// Prepare outputs to the configured band count.
			outputs.cochlear_frame.envelope.set_size(config.num_bands);
			outputs.cochlear_frame.fine_phase.set_size(config.num_bands);
			outputs.cochlear_frame.modulation_power.set_size(config.num_bands);
			outputs.cochlear_frame.band_center_hz.set_size(config.num_bands);

			// Build all analysis state.
			CochlearTransform::build_window(state.get());
			CochlearTransform::plan_fft(state.get());
			CochlearTransform::build_erb_bands(config, state.get());
			CochlearTransform::build_env_filters(config, state.get());
			CochlearTransform::reset_state(state.get());
		}

		void tick(const TickInfo&)
		{
			// Stream audio in.
			if (!inputs.mono.samples.empty())
			{
				CochlearTransform::push_samples(inputs.mono.samples.data(), inputs.mono.samples.size(), config, state.get());
			}

			// Propagate timestamp regardless.
			outputs.cochlear_frame.timestamp = inputs.mono.timestamp;

			// Build next frame if possible and analyze.
			if (!CochlearTransform::make_frame_from_ring(state.get()))
			{
				return;
			}

			CochlearTransform::analyze_one_frame(config, state.get(), outputs.cochlear_frame);
		}
	};
} // namespace robotick

#endif // ROBOTICK_PLATFORM_DESKTOP || ROBOTICK_PLATFORM_LINUX
