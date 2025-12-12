// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0
//
// HarmonicPitchWorkload.cpp  (thin wrapper around robotick::HarmonicPitch)

#include "robotick/api.h"
#include "robotick/systems/audio/AudioSystem.h"
#include "robotick/systems/auditory/CochlearFrame.h"
#include "robotick/systems/auditory/HarmonicPitch.h"
#include "robotick/systems/auditory/HarmonicPitchStabilizer.h"

#include <cstring>
#include <fstream>

namespace robotick
{
	struct HarmonicPitchConfig
	{
		HarmonicPitchSettings settings;
		uint32_t warmup_frame_count = 4;
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
		HarmonicPitchResult prev_detection = {};
		HarmonicPitchStabilizer stabilizer;
	};

	struct HarmonicPitchWorkload
	{
		HarmonicPitchConfig config;
		HarmonicPitchInputs inputs;
		HarmonicPitchOutputs outputs;

		State<HarmonicPitchState> state;

		void tick(const TickInfo&)
		{
			// analyse our current CochlearFrame
			HarmonicPitchResult found_result{};
			const HarmonicPitchResult prev_valid = state->prev_detection;
			const bool has_found = HarmonicPitch::find_or_continue_harmonic_features(
				config.settings, inputs.cochlear_frame.band_center_hz, inputs.cochlear_frame.envelope, prev_valid, found_result);

			HarmonicPitchResult stabilized{};
			HarmonicPitchStabilizerConfig stabilizer_config{};
			stabilizer_config.warmup_frame_count = config.warmup_frame_count;
			stabilizer_config.max_hold_frames = config.settings.max_hold_frames;
			state->stabilizer.configure(stabilizer_config);

			if (has_found)
			{
				state->prev_detection = found_result;
				if (state->stabilizer.process_valid_frame(found_result, stabilized))
				{
					outputs.pitch_info = stabilized;
				}
				else
				{
					outputs.pitch_info = HarmonicPitchResult{};
				}
			}
			else if (state->stabilizer.process_missing_frame(stabilized))
			{
				outputs.pitch_info = stabilized;
			}
			else
			{
				outputs.pitch_info = {};
				state->prev_detection = {};
			}

			// If we generated an empty pitch_info the struct will already be default constructed.
		}
	};

} // namespace robotick
