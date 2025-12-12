// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "robotick/api.h"
#include "robotick/framework/containers/FixedVector.h"
#include "robotick/systems/auditory/HarmonicPitch.h"

namespace robotick
{
	struct HarmonicPitchStabilizerConfig
	{
		uint32_t warmup_frame_count = 4;
		uint32_t max_hold_frames = 3;
	};

	class HarmonicPitchStabilizer
	{
	  public:
		void configure(const HarmonicPitchStabilizerConfig& cfg)
		{
			config.warmup_frame_count = (cfg.warmup_frame_count == 0) ? 1u : cfg.warmup_frame_count;
			if (config.warmup_frame_count > max_warmup_capacity)
			{
				config.warmup_frame_count = max_warmup_capacity;
			}
			config.max_hold_frames = cfg.max_hold_frames;
		}

		void reset()
		{
			warmup_buffer.clear();
			warmup_complete = false;
			missed_frames = 0;
			last_output = HarmonicPitchResult{};
		}

		bool process_valid_frame(const HarmonicPitchResult& result, HarmonicPitchResult& out_result)
		{
			missed_frames = 0;

			if (!warmup_complete)
			{
				warmup_buffer.add(result);
				last_output = average_buffer();
				out_result = last_output;

				if (warmup_buffer.size() >= config.warmup_frame_count)
				{
					warmup_complete = true;
					warmup_buffer.clear();
				}

				return true;
			}

			last_output = result;
			out_result = result;
			return true;
		}

		bool process_missing_frame(HarmonicPitchResult& out_result)
		{
			if (!warmup_complete)
			{
				if (!warmup_buffer.empty() && missed_frames < config.max_hold_frames)
				{
					++missed_frames;
					out_result = last_output;
					return true;
				}

				reset();
				out_result = HarmonicPitchResult{};
				return false;
			}

			if (missed_frames < config.max_hold_frames)
			{
				++missed_frames;
				out_result = last_output;
				return true;
			}

			reset();
			out_result = HarmonicPitchResult{};
			return false;
		}

		bool is_segment_active() const { return warmup_complete; }

	  private:
		static constexpr uint32_t max_warmup_capacity = 8;

		HarmonicPitchResult average_buffer() const
		{
			HarmonicPitchResult average{};
			if (warmup_buffer.empty())
			{
				return average;
			}

			const float inv = 1.0f / static_cast<float>(warmup_buffer.size());
			size_t max_harmonics = 0;
			for (size_t i = 0; i < warmup_buffer.size(); ++i)
			{
				max_harmonics = robotick::max(max_harmonics, warmup_buffer[i].harmonic_amplitudes.size());
			}

			for (size_t i = 0; i < max_harmonics; ++i)
			{
				average.harmonic_amplitudes.add(0.0f);
			}

			for (size_t i = 0; i < warmup_buffer.size(); ++i)
			{
				const HarmonicPitchResult& sample = warmup_buffer[i];
				average.h1_f0_hz += sample.h1_f0_hz;

				for (size_t h = 0; h < sample.harmonic_amplitudes.size(); ++h)
				{
					average.harmonic_amplitudes[h] += sample.harmonic_amplitudes[h];
				}
			}

			average.h1_f0_hz *= inv;
			for (size_t h = 0; h < average.harmonic_amplitudes.size(); ++h)
			{
				average.harmonic_amplitudes[h] *= inv;
			}

			return average;
		}

		HarmonicPitchStabilizerConfig config{};
		FixedVector<HarmonicPitchResult, max_warmup_capacity> warmup_buffer;
		bool warmup_complete = false;
		uint32_t missed_frames = 0;
		HarmonicPitchResult last_output{};
	};

} // namespace robotick
