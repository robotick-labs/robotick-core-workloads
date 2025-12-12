// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cmath>

namespace robotick
{
	struct ProsodyLinkConstraints
	{
		float max_jump_hz = 120.0f;
		float max_slope_hz_per_sec = 800.0f;
		float min_link_rms = 0.01f;
		float min_link_confidence = 0.3f;
	};

	struct ProsodyLinkSample
	{
		float pitch_hz = 0.0f;
		float rms = 0.0f;
		float confidence = 0.0f;
		float time_sec = 0.0f;

		bool is_valid() const { return pitch_hz > 0.0f; }
	};

	struct ProsodyLinkEvaluation
	{
		bool connect = false;
		float link_rms = 0.0f;
	};

	inline ProsodyLinkEvaluation evaluate_prosody_link(
		const ProsodyLinkConstraints& constraints, const ProsodyLinkSample& previous, const ProsodyLinkSample& current)
	{
		ProsodyLinkEvaluation eval{};
		if (!previous.is_valid() || !current.is_valid())
		{
			return eval;
		}

		const float dt = (current.time_sec > previous.time_sec) ? (current.time_sec - previous.time_sec) : 1e-3f;
		const float pitch_jump = fabsf(current.pitch_hz - previous.pitch_hz);
		const float slope = pitch_jump / dt;
		const float rms_avg = 0.5f * (previous.rms + current.rms);
		const float confidence_avg = 0.5f * (previous.confidence + current.confidence);

		if (pitch_jump > constraints.max_jump_hz)
		{
			return eval;
		}

		if (slope > constraints.max_slope_hz_per_sec)
		{
			return eval;
		}

		if (rms_avg < constraints.min_link_rms)
		{
			return eval;
		}

		if (confidence_avg < constraints.min_link_confidence)
		{
			return eval;
		}

		eval.connect = true;
		eval.link_rms = rms_avg;
		return eval;
	}
} // namespace robotick
