// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#if defined(ROBOTICK_PLATFORM_DESKTOP) || defined(ROBOTICK_PLATFORM_LINUX)

#include "robotick/api.h"
#include "robotick/framework/strings/FixedString.h"
#include "robotick/systems/auditory/ProsodyFusion.h"
#include "robotick/systems/auditory/ProsodyState.h"
#include "robotick/systems/auditory/SpeechToText.h"

#include <cmath>
#include <cstring>

namespace robotick
{
	// Stores how aggressively we keep/densify history before passing it to UI.
	struct ProsodyFusionConfig
	{
		float history_duration_sec = 8.0f;	   // rolling buffer length for live curves
		uint32_t simplified_sample_count = 16; // downsample count per segment
		float minimum_segment_duration_sec = 0.1f;
	};

	struct ProsodyFusionInputs
	{
		ProsodyState prosody_state;
		Transcript proto_transcript;
		Transcript transcript;
	};

	struct ProsodyFusionOutputs
	{
		ProsodicSegment current_segment;
		ProsodicSegmentBuffer prev_segments;
	};

	// Keeps the rolling prosody buffer plus the last transcript metadata to
	// detect updates. Each entry stores a wall-clock so we can stitch timelines
	// together without assuming a fixed tick rate.
	struct ProsodyFusionState
	{
		ProsodyHistoryBuffer history;

		float last_proto_start = -1.0f;
		float last_proto_duration = -1.0f;
		FixedString512 last_proto_text;

		float last_final_start = -1.0f;
		float last_final_duration = -1.0f;
		FixedString512 last_final_text;
	};

	struct ProsodyFusionWorkload
	{
		ProsodyFusionConfig config;
		ProsodyFusionInputs inputs;
		ProsodyFusionOutputs outputs;
		StatePtr<ProsodyFusionState> state;

		void start(float /*tick_rate_hz*/)
		{
			state->history.clear();
			state->last_proto_text.clear();
			state->last_final_text.clear();
			outputs.current_segment = ProsodicSegment{};
			outputs.prev_segments.clear();
		}

		static bool transcript_has_content(const Transcript& transcript) { return (!transcript.text.empty()) && (transcript.duration_sec > 0.0f); }

		// Whisper can resend proto transcripts without changing the text. This
		// helper filters out redundant notifications so we only emit segments
		// when timing or content changed.
		static bool transcript_changed(const Transcript& transcript, float& last_start, float& last_duration, FixedString512& last_text)
		{
			if (!transcript_has_content(transcript))
			{
				return false;
			}

			const bool text_changed = !(last_text == transcript.text.c_str());
			const bool start_changed = fabsf(transcript.start_time_sec - last_start) > 1e-3f;
			const bool duration_changed = fabsf(transcript.duration_sec - last_duration) > 1e-3f;

			if (text_changed || start_changed || duration_changed)
			{
				last_start = transcript.start_time_sec;
				last_duration = transcript.duration_sec;
				last_text = transcript.text.c_str();
				return true;
			}

			return false;
		}

		// Push the newest ProsodyState into the rolling history, trimming any
		// entries that fall outside the configured window.
		// --- history maintenance -------------------------------------------------

		void append_history_sample(const ProsodyState& prosody_state, const float time_now)
		{
			if (state->history.full())
			{
				drop_oldest_history(state->history, 1);
			}

			// Keep an ordered list of {timestamp, state}
			state->history.add(ProsodyHistorySample{time_now, prosody_state});

			const float min_time = time_now - config.history_duration_sec;
			// Determine how many stale entries sit outside the rolling window.
			size_t drop_count = 0;
			for (size_t i = 0; i < state->history.size(); ++i)
			{
				if (state->history[i].time_sec >= min_time)
				{
					break;
				}
				++drop_count;
			}

			if (drop_count > 0)
			{
				// Slide the buffer down in one memmove instead of popping.
				drop_oldest_history(state->history, drop_count);
			}
		}

		// Interpolates the stored history at an arbitrary timestamp. We fall
		// back to the latest sample if the requested time is ahead of history.
		bool sample_history(const float time_sec, ProsodyState& out) const
		{
			if (state->history.empty())
			{
				return false;
			}

			if (time_sec <= state->history[0].time_sec)
			{
				// Before oldest sample → clamp to oldest entry
				out = state->history[0].prosody;
				return true;
			}

			for (size_t i = 0; i + 1 < state->history.size(); ++i)
			{
				const ProsodyHistorySample& a = state->history[i];
				const ProsodyHistorySample& b = state->history[i + 1];
				if (time_sec >= a.time_sec && time_sec <= b.time_sec)
				{
					const float span = b.time_sec - a.time_sec;
					const float alpha = (span > 1e-6f) ? (time_sec - a.time_sec) / span : 0.0f;
					// Linear interpolation keeps the curve smooth enough for UI
					out.pitch_hz = a.prosody.pitch_hz + (b.prosody.pitch_hz - a.prosody.pitch_hz) * alpha;
					out.rms = a.prosody.rms + (b.prosody.rms - a.prosody.rms) * alpha;
					out.voiced_confidence = a.prosody.voiced_confidence + (b.prosody.voiced_confidence - a.prosody.voiced_confidence) * alpha;
					return true;
				}
			}

			// Beyond newest sample → clamp to most recent
			out = state->history[state->history.size() - 1].prosody;
			return true;
		}

		// Builds a "live" segment by sampling the raw history between the given
		// times. These segments purposely omit text so UIs can render tone-only
		// previews before Whisper emits words.

		bool build_segment_from_history_window(float start_time, float end_time, ProsodicSegment& out_segment)
		{
			if (state->history.size() < 2)
			{
				return false;
			}

			const float history_start = state->history[0].time_sec;
			const float history_end = state->history[state->history.size() - 1].time_sec;
			const float clamped_start = robotick::max(start_time, history_start);
			const float clamped_end = robotick::min(end_time, history_end);
			if (!(clamped_end > clamped_start))
			{
				return false;
			}

			out_segment = ProsodicSegment{};
			out_segment.start_time_sec = clamped_start;
			out_segment.end_time_sec = clamped_end;
			out_segment.is_finalised = false;
			out_segment.words.clear();

			const uint32_t sample_count = robotick::max<uint32_t>(2u, config.simplified_sample_count);
			float confidence_sum = 0.0f;
			int confidence_count = 0;

			for (uint32_t i = 0; i < sample_count; ++i)
			{
				const float alpha = (sample_count <= 1) ? 0.0f : static_cast<float>(i) / static_cast<float>(sample_count - 1);
				// Evenly spaced sampling gives a normalized curve regardless of
				// how long the underlying segment lasted.
				const float sample_time = clamped_start + alpha * (clamped_end - clamped_start);

				ProsodyState sampled_state{};
				if (!sample_history(sample_time, sampled_state))
				{
					sampled_state = inputs.prosody_state;
				}

				if (!out_segment.pitch_hz.full())
				{
					out_segment.pitch_hz.add(sampled_state.pitch_hz);
				}

				if (!out_segment.rms.full())
				{
					out_segment.rms.add(sampled_state.rms);
				}

				confidence_sum += sampled_state.voiced_confidence;
				++confidence_count;
			}

			if (confidence_count > 0)
			{
				out_segment.mean_voiced_confidence = confidence_sum / static_cast<float>(confidence_count);
			}

			return true;
		}

		// Converts a proto/final Whisper transcript to a segment and samples the
		// matching prosody timeline. `is_finalised` differentiates proto vs final.
		bool build_segment_from_transcript(const Transcript& transcript, const bool is_finalised, ProsodicSegment& out_segment)
		{
			if (!transcript_has_content(transcript))
			{
				return false;
			}

			// Clamp duration so even short utterances have enough samples to draw.
			const float duration = robotick::max(config.minimum_segment_duration_sec, transcript.duration_sec);
			const float start_time = transcript.start_time_sec;
			const float end_time = start_time + duration;

			out_segment = ProsodicSegment{};
			out_segment.start_time_sec = start_time;
			out_segment.end_time_sec = end_time;
			out_segment.is_finalised = is_finalised;
			out_segment.words.clear();
			// Copy any Whisper word timings that intersect the segment window.
			for (const TranscribedWord& word : transcript.words)
			{
				if (word.end_time_sec < start_time || word.start_time_sec > end_time)
				{
					continue;
				}
				if (out_segment.words.full())
				{
					break;
				}
				out_segment.words.add(word);
			}

			const uint32_t sample_count = robotick::max<uint32_t>(2u, config.simplified_sample_count);

			float confidence_sum = 0.0f;
			int confidence_count = 0;

			for (uint32_t i = 0; i < sample_count; ++i)
			{
				const float alpha = (sample_count <= 1) ? 0.0f : static_cast<float>(i) / static_cast<float>(sample_count - 1);
				const float sample_time = start_time + alpha * (end_time - start_time);

				ProsodyState sampled_state{};
				// Fall back to the latest tick's state if the history buffer has a gap.
				if (!sample_history(sample_time, sampled_state))
				{
					sampled_state = inputs.prosody_state;
				}

				if (!out_segment.pitch_hz.full())
				{
					out_segment.pitch_hz.add(sampled_state.pitch_hz);
				}

				if (!out_segment.rms.full())
				{
					out_segment.rms.add(sampled_state.rms);
				}

				confidence_sum += sampled_state.voiced_confidence;
				++confidence_count;
			}

			if (confidence_count > 0)
			{
				// Expose how "voiced" the segment felt so downstream logic can
				// differentiate strong vs weak speech.
				out_segment.mean_voiced_confidence = confidence_sum / static_cast<float>(confidence_count);
			}

			return true;
		}

		// Main fusion loop: keep the history up to date, emit a live segment
		// every frame, and push proto/final segments when Whisper updates.

		void tick(const TickInfo& tick_info)
		{
			append_history_sample(inputs.prosody_state, tick_info.time_now);

			// Live stream: always emit the most recent window of prosody so UI
			// can show tone even without transcripts.
			ProsodicSegment live_segment;
			const float live_end_time = state->history.empty() ? tick_info.time_now : state->history[state->history.size() - 1].time_sec;
			const float live_start_time = live_end_time - config.history_duration_sec;
			if (build_segment_from_history_window(live_start_time, live_end_time, live_segment))
			{
				outputs.current_segment = live_segment;
			}
			else
			{
				outputs.current_segment = ProsodicSegment{};
			}

			// Proto segment: Whisper is mid-sentence. We surface it immediately
			// so higher layers can update overlays/text continuously.
			if (transcript_changed(inputs.proto_transcript, state->last_proto_start, state->last_proto_duration, state->last_proto_text))
			{
				ProsodicSegment proto_segment;
				if (build_segment_from_transcript(inputs.proto_transcript, false, proto_segment))
				{
					outputs.current_segment = proto_segment;
				}
			}

			// Finalized segment: replace the proto entry and append to the
			// baked history, preserving the exact timings Whisper confirmed.
			if (transcript_changed(inputs.transcript, state->last_final_start, state->last_final_duration, state->last_final_text))
			{
				ProsodicSegment baked_segment;
				if (build_segment_from_transcript(inputs.transcript, true, baked_segment))
				{
					append_segment_with_capacity(outputs.prev_segments, baked_segment);
					outputs.current_segment = baked_segment;
				}
			}
		}

		void stop()
		{
			state->history.clear();
			state->last_proto_text.clear();
			state->last_final_text.clear();
		}
	};

} // namespace robotick

#endif // ROBOTICK_PLATFORM_DESKTOP || ROBOTICK_PLATFORM_LINUX
