// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#include "robotick/systems/auditory/ProsodyFusion.h"

#include "robotick/api.h"

namespace robotick
{
	ROBOTICK_REGISTER_STRUCT_BEGIN(ProsodyHistorySample)
	ROBOTICK_STRUCT_FIELD(ProsodyHistorySample, float, time_sec)
	ROBOTICK_STRUCT_FIELD(ProsodyHistorySample, ProsodyState, prosody)
	ROBOTICK_REGISTER_STRUCT_END(ProsodyHistorySample)

	ROBOTICK_REGISTER_FIXED_VECTOR(ProsodyPitchCurve, float);
	ROBOTICK_REGISTER_FIXED_VECTOR(ProsodyPitchMask, uint8_t);
	ROBOTICK_REGISTER_FIXED_VECTOR(ProsodyRmsCurve, float);

	ROBOTICK_REGISTER_ENUM_BEGIN(ProsodicSegmentState)
	ROBOTICK_ENUM_VALUE("Ongoing", ProsodicSegmentState::Ongoing)
	ROBOTICK_ENUM_VALUE("Completed", ProsodicSegmentState::Completed)
	ROBOTICK_ENUM_VALUE("Finalised", ProsodicSegmentState::Finalised)
	ROBOTICK_REGISTER_ENUM_END(ProsodicSegmentState)

	ROBOTICK_REGISTER_STRUCT_BEGIN(ProsodicSegment)
	ROBOTICK_STRUCT_FIELD(ProsodicSegment, float, start_time_sec)
	ROBOTICK_STRUCT_FIELD(ProsodicSegment, float, end_time_sec)
	ROBOTICK_STRUCT_FIELD(ProsodicSegment, ProsodyPitchCurve, pitch_hz)
	ROBOTICK_STRUCT_FIELD(ProsodicSegment, ProsodyRmsCurve, rms)
	ROBOTICK_STRUCT_FIELD(ProsodicSegment, ProsodyPitchMask, pitch_link_mask)
	ROBOTICK_STRUCT_FIELD(ProsodicSegment, ProsodyRmsCurve, link_rms)
	ROBOTICK_STRUCT_FIELD(ProsodicSegment, float, mean_voiced_confidence)
	ROBOTICK_STRUCT_FIELD(ProsodicSegment, ProsodicSegmentState, state)
	ROBOTICK_STRUCT_FIELD(ProsodicSegment, TranscribedWords, words)
	ROBOTICK_REGISTER_STRUCT_END(ProsodicSegment)

	ROBOTICK_REGISTER_FIXED_VECTOR(ProsodyHistoryBuffer, ProsodyHistorySample);
	ROBOTICK_REGISTER_FIXED_VECTOR(ProsodicSegmentBuffer, ProsodicSegment);

	// Sliding-window helper shared by the workload: keeps the newest samples
	// while avoiding reallocations on every tick.
	void drop_oldest_history(ProsodyHistoryBuffer& buffer, size_t count)
	{
		if (count == 0 || buffer.empty())
		{
			return;
		}

		if (count >= buffer.size())
		{
			buffer.set_size(0);
			return;
		}

		const size_t keep = buffer.size() - count;
		::memmove(buffer.data(), buffer.data() + count, keep * sizeof(ProsodyHistorySample));
		buffer.set_size(keep);
	}

	void drop_oldest_segments(ProsodicSegmentBuffer& buffer, size_t count)
	{
		if (count == 0 || buffer.empty())
		{
			return;
		}

		if (count >= buffer.size())
		{
			buffer.set_size(0);
			return;
		}

		const size_t keep = buffer.size() - count;
		::memmove(buffer.data(), buffer.data() + count, keep * sizeof(ProsodicSegment));
		buffer.set_size(keep);
	}

	void append_segment_with_capacity(ProsodicSegmentBuffer& buffer, const ProsodicSegment& segment)
	{
		if (buffer.full())
		{
			drop_oldest_segments(buffer, 1);
		}
		buffer.add(segment);
	}

} // namespace robotick
