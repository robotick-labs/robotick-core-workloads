// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "robotick/framework/containers/FixedVector.h"
#include "robotick/systems/auditory/ProsodyState.h"
#include "robotick/systems/auditory/SpeechToText.h"

namespace robotick
{
	// Compact representation of a segment's F0 contour and RMS envelope.
	using ProsodyPitchCurve = FixedVector<float, 128>;
	using ProsodyRmsCurve = FixedVector<float, 128>;

	struct ProsodyHistorySample
	{
		float time_sec = 0.0f;
		ProsodyState prosody;
	};

	using ProsodyHistoryBuffer = FixedVector<ProsodyHistorySample, 4096>;

	// Downsampled view of prosody spanning a speech segment. Words carry the
	// transcript text (proto or finalized) so we do not need a separate string.
	struct ProsodicSegment
	{
		float start_time_sec = 0.0f;
		float end_time_sec = 0.0f;

		ProsodyPitchCurve pitch_hz;
		ProsodyRmsCurve rms;

		float mean_voiced_confidence = 0.0f;
		bool is_finalised = false;

		TranscribedWords words;
	};

	using ProsodicSegmentBuffer = FixedVector<ProsodicSegment, 32>;

	void drop_oldest_history(ProsodyHistoryBuffer& buffer, size_t count);
	void drop_oldest_segments(ProsodicSegmentBuffer& buffer, size_t count);
	void append_segment_with_capacity(ProsodicSegmentBuffer& buffer, const ProsodicSegment& segment);

} // namespace robotick
