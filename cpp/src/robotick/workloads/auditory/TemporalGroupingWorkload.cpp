// Copyright Robotick
// SPDX-License-Identifier: Apache-2.0
//
// TemporalGroupingWorkload.cpp  (thin wrapper around robotick::TemporalGrouping)

#include "robotick/api.h"
#include "robotick/systems/audio/AudioSystem.h"
#include "robotick/systems/auditory/CochlearFrame.h"
#include "robotick/systems/auditory/SourceCandidate.h"
#include "robotick/systems/auditory/TemporalGrouping.h"

#include <cmath>
#include <cstring>
#include <fstream>

namespace robotick
{
	struct TemporalGroupingConfig
	{
		TemporalGroupingSettings settings;
	};

	struct TemporalGroupingInputs
	{
		CochlearFrame cochlear_frame;
	};

	struct TemporalGroupingOutputs
	{
		SourceCandidates8 source_candidates;
		SourceCandidate first_source;
	};

	struct TemporalGroupingWorkload
	{
		TemporalGroupingConfig config;
		TemporalGroupingInputs inputs;
		TemporalGroupingOutputs outputs;

		void tick(const TickInfo& tick_info)
		{
			(void)tick_info;

			// reset everything in case we don't find anything
			outputs.source_candidates.clear();
			outputs.first_source = SourceCandidate{};

			// analyse out current CochlearFrame
			const CochlearFrame& cochlear_frame = inputs.cochlear_frame;

			TemporalGroupingResult result{};
			const int found_band_id =
				TemporalGrouping::find_strongest_f0_band_id(config.settings, cochlear_frame.band_center_hz, cochlear_frame.envelope, result);

			// process & store our results to outputs
			if (found_band_id >= 0)
			{
				SourceCandidate out{};
				out.pitch_hz = result.h1_f0_hz;
				out.amplitude = result.h1_amplitude;
				out.centroid_freq_hz = result.h1_f0_hz;

				outputs.first_source = out;
				outputs.source_candidates.add(out);
			}
		}
	};

} // namespace robotick