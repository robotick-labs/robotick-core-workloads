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

	struct HarmonicPitchWorkload
	{
		HarmonicPitchConfig config;
		HarmonicPitchInputs inputs;
		HarmonicPitchOutputs outputs;

		void tick(const TickInfo& tick_info)
		{
			(void)tick_info;

			// reset everything in case we don't find anything
			outputs.source_candidates.clear();
			outputs.first_source = SourceCandidate{};

			// analyse out current CochlearFrame
			const CochlearFrame& cochlear_frame = inputs.cochlear_frame;

			HarmonicPitchResult result{};
			const int found_band_id =
				HarmonicPitch::find_harmonic_features(config.settings, cochlear_frame.band_center_hz, cochlear_frame.envelope, result);

			// process & store our results to outputs
			if (found_band_id >= 0)
			{
				SourceCandidate out{};
				out.h1_f0_hz = result.h1_f0_hz;
				out.harmonic_amplitudes = result.harmonic_amplitudes;

				outputs.first_source = out;
				outputs.source_candidates.add(out);
			}
		}
	};

} // namespace robotick