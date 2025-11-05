// Copyright Robotick
// SPDX-License-Identifier: Apache-2.0
//
// HarmonicPitchWorkload.cpp  (thin wrapper around robotick::HarmonicPitch)

#include "robotick/api.h"
#include "robotick/systems/audio/AudioSystem.h"
#include "robotick/systems/auditory/CochlearFrame.h"
#include "robotick/systems/auditory/HarmonicPitch.h"
#include "robotick/systems/auditory/SourceCandidate.h"

#include <cmath>
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
		SourceCandidates8 source_candidates;
		SourceCandidate first_source;
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
			outputs.source_candidates.clear();
			outputs.first_source = SourceCandidate{};

			// analyse out current CochlearFrame
			const CochlearFrame& cochlear_frame = inputs.cochlear_frame;

			HarmonicPitchResult found_result{};
			const bool has_found = HarmonicPitch::find_or_continue_harmonic_features(
				config.settings, cochlear_frame.band_center_hz, cochlear_frame.envelope, state->prev_result, found_result);

			// process & store our results to outputs
			if (has_found)
			{
				SourceCandidate out{};
				out.h1_f0_hz = found_result.h1_f0_hz;
				out.harmonic_amplitudes = found_result.harmonic_amplitudes;

				outputs.first_source = out;
				outputs.source_candidates.add(out);
			}

			state->prev_result = found_result;
		}
	};

} // namespace robotick