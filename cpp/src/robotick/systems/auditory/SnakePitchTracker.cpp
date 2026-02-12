// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#include "robotick/systems/auditory/SnakePitchTracker.h"

#include "robotick/api.h"

#include <cmath>

namespace robotick
{
	ROBOTICK_REGISTER_STRUCT_BEGIN(SnakePitchTrackerConfig)
	ROBOTICK_STRUCT_FIELD(SnakePitchTrackerConfig, float, min_peak_amplitude)
	ROBOTICK_STRUCT_FIELD(SnakePitchTrackerConfig, float, peak_merge_cents)
	ROBOTICK_STRUCT_FIELD(SnakePitchTrackerConfig, float, snake_match_cents)
	ROBOTICK_STRUCT_FIELD(SnakePitchTrackerConfig, uint32_t, snake_keep_alive_frames)
	ROBOTICK_STRUCT_FIELD(SnakePitchTrackerConfig, float, harmonic_match_cents)
	ROBOTICK_STRUCT_FIELD(SnakePitchTrackerConfig, uint32_t, max_snakes)
	ROBOTICK_REGISTER_STRUCT_END(SnakePitchTrackerConfig)

	SnakePitchTracker::SnakePitchTracker() = default;

	void SnakePitchTracker::configure(const SnakePitchTrackerConfig& cfg)
	{
		config_ = cfg;
		reset();
	}

	void SnakePitchTracker::reset()
	{
		snakes_.clear();
	}

	bool SnakePitchTracker::update(const CochlearFrame& frame, HarmonicPitchResult& out_result)
	{
		FixedVector<Peak, 128> peaks;
		detect_peaks(frame, peaks);
		update_snakes(frame, peaks);

		if (find_harmonic_set(out_result))
		{
			return true;
		}

		out_result = HarmonicPitchResult{};
		return false;
	}

	float SnakePitchTracker::hz_to_cents(float a, float b)
	{
		if (!(a > 0.0f && b > 0.0f))
		{
			return 1e6f;
		}
		return 1200.0f * fabsf(log2f(a / b));
	}

	size_t SnakePitchTracker::find_nearest_band(const CochlearFrame& frame, float freq)
	{
		const size_t band_count = frame.band_center_hz.size();
		if (band_count == 0)
		{
			return 0;
		}

		size_t best_idx = 0;
		float best_diff = fabsf(frame.band_center_hz[0] - freq);
		for (size_t i = 1; i < band_count; ++i)
		{
			const float diff = fabsf(frame.band_center_hz[i] - freq);
			if (diff < best_diff)
			{
				best_diff = diff;
				best_idx = i;
			}
		}
		return best_idx;
	}

	void SnakePitchTracker::center_snake_on_local_peak(const CochlearFrame& frame, SnakeTrack& snake)
	{
		const size_t band_count = frame.envelope.size();
		if (band_count == 0)
		{
			return;
		}

		size_t idx = find_nearest_band(frame, snake.freq_hz);
		bool improved = true;
		while (improved)
		{
			improved = false;
			const float current = frame.envelope[idx];
			size_t best_idx = idx;
			float best_val = current;

			if (idx > 0 && frame.envelope[idx - 1] > best_val)
			{
				best_val = frame.envelope[idx - 1];
				best_idx = idx - 1;
			}
			if (idx + 1 < band_count && frame.envelope[idx + 1] > best_val)
			{
				best_val = frame.envelope[idx + 1];
				best_idx = idx + 1;
			}

			if (best_idx != idx && best_val > current)
			{
				idx = best_idx;
				improved = true;
			}
		}

		snake.freq_hz = frame.band_center_hz[idx];
		snake.amplitude = frame.envelope[idx];
	}

	void SnakePitchTracker::detect_peaks(const CochlearFrame& frame, FixedVector<Peak, 128>& out_peaks) const
	{
		out_peaks.clear();
		const size_t band_count = frame.envelope.size();
		if (band_count < 3)
		{
			return;
		}

		for (size_t i = 1; i + 1 < band_count; ++i)
		{
			const float prev = frame.envelope[i - 1];
			const float curr = frame.envelope[i];
			const float next = frame.envelope[i + 1];
			if (curr < config_.min_peak_amplitude || !(curr > prev && curr >= next))
			{
				continue;
			}

			Peak peak{};
			peak.freq = frame.band_center_hz[i];
			peak.amplitude = curr;

			bool merged = false;
			for (size_t existing = 0; existing < out_peaks.size(); ++existing)
			{
				if (hz_to_cents(out_peaks[existing].freq, peak.freq) <= config_.peak_merge_cents)
				{
					if (peak.amplitude > out_peaks[existing].amplitude)
					{
						out_peaks[existing] = peak;
					}
					merged = true;
					break;
				}
			}

			if (!merged && !out_peaks.full())
			{
				out_peaks.add(peak);
			}
		}
	}

	void SnakePitchTracker::update_snakes(const CochlearFrame& frame, const FixedVector<Peak, 128>& peaks)
	{
		FixedVector<uint8_t, 128> peak_used;
		peak_used.set_size(peaks.size());
		for (size_t i = 0; i < peak_used.size(); ++i)
		{
			peak_used[i] = 0;
		}

		for (size_t snake_idx = 0; snake_idx < snakes_.size();)
		{
			SnakeTrack& snake = snakes_[snake_idx];
			int best_peak = -1;
			float best_cents = 1e6f;
			for (size_t peak_idx = 0; peak_idx < peaks.size(); ++peak_idx)
			{
				if (peak_used[peak_idx])
				{
					continue;
				}
				const float cents = hz_to_cents(snake.freq_hz, peaks[peak_idx].freq);
				if (cents <= config_.snake_match_cents && cents < best_cents)
				{
					best_cents = cents;
					best_peak = static_cast<int>(peak_idx);
				}
			}

			if (best_peak >= 0)
			{
				const Peak& peak = peaks[static_cast<size_t>(best_peak)];
				snake.freq_hz = peak.freq;
				snake.amplitude = peak.amplitude;
				snake.keep_alive = config_.snake_keep_alive_frames;
				peak_used[static_cast<size_t>(best_peak)] = 1;
				center_snake_on_local_peak(frame, snake);
				++snake_idx;
			}
			else
			{
				if (snake.keep_alive > 0)
				{
					--snake.keep_alive;
					center_snake_on_local_peak(frame, snake);
					++snake_idx;
				}
				else
				{
					const size_t last_index = snakes_.size() - 1;
					if (snake_idx != last_index)
					{
						snakes_[snake_idx] = snakes_[last_index];
					}
					snakes_.set_size(last_index);
				}
			}
		}

		for (size_t peak_idx = 0; peak_idx < peaks.size(); ++peak_idx)
		{
			if (peak_used[peak_idx])
			{
				continue;
			}

			if (snakes_.full() || snakes_.size() >= config_.max_snakes)
			{
				break;
			}

			SnakeTrack track{};
			track.freq_hz = peaks[peak_idx].freq;
			track.amplitude = peaks[peak_idx].amplitude;
			track.keep_alive = config_.snake_keep_alive_frames;
			snakes_.add(track);
			center_snake_on_local_peak(frame, snakes_[snakes_.size() - 1]);
		}
	}

	bool SnakePitchTracker::find_harmonic_set(HarmonicPitchResult& out_result) const
	{
		if (snakes_.empty())
		{
			return false;
		}

		float best_score = 0.0f;
		HarmonicPitchResult best{};

		for (const SnakeTrack& base_snake : snakes_)
		{
			if (base_snake.freq_hz <= 0.0f)
			{
				continue;
			}

			FixedVector<float, harmonic_pitch::MaxHarmonics> amplitudes;
			amplitudes.clear();
			float score = 0.0f;

			FixedVector<uint8_t, 64> used_snakes;
			used_snakes.set_size(snakes_.size());
			for (size_t i = 0; i < used_snakes.size(); ++i)
			{
				used_snakes[i] = 0;
			}

			for (size_t harmonic_id = 1; harmonic_id <= harmonic_pitch::MaxHarmonics; ++harmonic_id)
			{
				const float target_freq = base_snake.freq_hz * static_cast<float>(harmonic_id);
				int best_idx = -1;
				float best_cents = 1e6f;

				for (size_t snake_idx = 0; snake_idx < snakes_.size(); ++snake_idx)
				{
					if (used_snakes[snake_idx])
					{
						continue;
					}
					const float cents = hz_to_cents(target_freq, snakes_[snake_idx].freq_hz);
					if (cents <= config_.harmonic_match_cents && cents < best_cents)
					{
						best_cents = cents;
						best_idx = static_cast<int>(snake_idx);
					}
				}

				float amplitude = 0.0f;
				if (best_idx >= 0)
				{
					amplitude = snakes_[static_cast<size_t>(best_idx)].amplitude;
					used_snakes[static_cast<size_t>(best_idx)] = 1;
					score += amplitude * (harmonic_id == 1 ? 1.5f : 1.0f);
				}
				else
				{
					score *= 0.98f;
				}

				amplitudes.add(amplitude);
			}

			if (score > best_score && amplitudes.size() > 0)
			{
				best_score = score;
				best.h1_f0_hz = base_snake.freq_hz;
				best.harmonic_amplitudes = amplitudes;
			}
		}

		if (best_score <= 0.0f || best.h1_f0_hz <= 0.0f)
		{
			return false;
		}

		while (best.harmonic_amplitudes.size() < harmonic_pitch::MaxHarmonics)
		{
			best.harmonic_amplitudes.add(0.0f);
		}

		out_result = best;
		return true;
	}

} // namespace robotick
