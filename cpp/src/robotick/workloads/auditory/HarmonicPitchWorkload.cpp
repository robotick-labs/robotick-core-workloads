// Copyright Robotick
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
	};

	struct HarmonicPitchWorkload
	{
		HarmonicPitchConfig config;
		HarmonicPitchInputs inputs;
		HarmonicPitchOutputs outputs;

		State<HarmonicPitchState> state;

		void tick(const TickInfo&)
		{
			// reset everything in case we don't find anything
			outputs.pitch_info = {};

			// analyse our current CochlearFrame
			HarmonicPitchResult found_result{};
			const bool has_found = HarmonicPitch::find_or_continue_harmonic_features(
				config.settings, inputs.cochlear_frame.band_center_hz, inputs.cochlear_frame.envelope, state->prev_result, found_result);

			if (has_found)
			{
				outputs.pitch_info = found_result;
			}

			state->prev_result = found_result;
		}
	};

} // namespace robotick
