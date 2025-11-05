// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0
//
// TemporalGrouping.h  (lean header: declarations only)

#pragma once

#include "robotick/api.h"
#include "robotick/framework/common/FixedVector.h"
#include "robotick/systems/audio/AudioFrame.h"

#include <cstdint>

namespace robotick
{
	namespace temporal_grouping
	{
		static constexpr size_t MaxHarmonics = 16;
	};

	struct TemporalGroupingSettings
	{
		// Selection / gating
		float min_amplitude = 0.3f;
		float reuse_penalty = 0.45f;
	};

	struct TemporalGroupingResult
	{
		float h1_f0_hz = 0.0f;
		float h1_amplitude = 0.0f;

		FixedVector<float, temporal_grouping::MaxHarmonics> harmonic_amplitudes;
	};

	class TemporalGrouping
	{
	  public:
		static int find_strongest_f0_band_id(
			const TemporalGroupingSettings& settings, const AudioBuffer128& centers, const AudioBuffer128& envelope, TemporalGroupingResult& result);
	};

} // namespace robotick
