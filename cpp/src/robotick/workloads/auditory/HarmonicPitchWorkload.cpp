// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0
//
// HarmonicPitchWorkload.cpp  (thin wrapper around robotick::HarmonicPitch)

#include "robotick/api.h"
#include "robotick/systems/audio/AudioSystem.h"
#include "robotick/systems/auditory/CochlearFrame.h"
#include "robotick/systems/auditory/HarmonicPitch.h"

#include <cstring>
#include <fstream>

namespace robotick
{
	struct HarmonicPitchConfig
	{
		HarmonicPitchSettings settings;
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
		HarmonicPitchResult prev_result = {};
		uint32_t missed_frames = 0;
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
			const HarmonicPitchResult prev_valid = state->prev_result;
			const bool has_found = HarmonicPitch::find_or_continue_harmonic_features(
				config.settings, inputs.cochlear_frame.band_center_hz, inputs.cochlear_frame.envelope, prev_valid, found_result);

			if (has_found)
			{
				outputs.pitch_info = found_result;
				state->prev_result = found_result;
				state->missed_frames = 0;
			}
			else if (prev_valid.h1_f0_hz > 0.0f && state->missed_frames < config.settings.max_hold_frames)
			{
				// Hold on to the last strong pitch for a few frames so downstream logic keeps continuity.
				outputs.pitch_info = prev_valid;
				state->missed_frames++;
			}
			else
			{
				outputs.pitch_info = {};
				state->prev_result = {};
				state->missed_frames = 0;
			}

			// If we generated an empty pitch_info the struct will already be default constructed.
		}
	};

} // namespace robotick
