// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "robotick/framework/containers/FixedVector.h"
#include "robotick/systems/auditory/CochlearFrame.h"
#include "robotick/systems/auditory/HarmonicPitch.h"

// SnakePitchTracker — snake-based Cochlear ridge tracker feeding the prosody pipeline.
//
// 1. Per-peak “snake” tracking (every tick)
//    - Detect local envelope peaks and smooth them.
//    - Each peak spawns or updates a short-lived snake that remembers freq/amplitude and hill-climbs
//      to the local summit so it stays centered on the “white ridge”.
//    - Each snake keeps a keep-alive counter so brief dropouts don’t kill obvious contours.
//
// 2. Harmonic inference from live snakes
//    - After updating snakes, evaluate which subsets form harmonic series (allowing missing members)
//      by comparing their frequencies within a cents tolerance.
//    - The strongest subset becomes the emitted HarmonicPitchResult (f0 + harmonic amplitudes),
//      so downstream workloads still see the same API.
//
// 3. Why it matters
//    - True f0 ridges survive formant crossings/consonant bursts because snakes track locally.
//    - Multiple ridges coexist, giving better voicing/confidence signals (unused snakes decay).
//    - Improvements land without changing the public API, so ProsodyAnalyser/Fusion immediately
//      benefit from the more stable pitch curve.

namespace robotick
{
	struct SnakePitchTrackerConfig
	{
		float min_peak_amplitude = 0.05f;	  // Envelope peaks must exceed this RMS-normalized amplitude to spawn a snake.
		float peak_merge_cents = 25.0f;		  // Peaks closer than this cents delta are merged so tiny wobble bands act as one ridge.
		float snake_match_cents = 100.0f;	  // How far a snake is allowed to jump between frames when reacquiring its ridge.
		uint32_t snake_keep_alive_frames = 4; // Drop a snake after N missed matches so short gaps do not instantly kill it.
		float harmonic_match_cents = 100.0f;  // Harmonic grouping tolerance when explaining snakes as f0 + harmonics.
		uint32_t max_snakes = 32;			  // Upper bound on live snakes to avoid pathological allocations.
	};

	struct SnakeTrack
	{
		float freq_hz = 0.0f;
		float amplitude = 0.0f;
		uint32_t keep_alive = 0;
	};

	class SnakePitchTracker
	{
	  public:
		SnakePitchTracker();

		void configure(const SnakePitchTrackerConfig& cfg);
		void reset();

		bool update(const CochlearFrame& frame, HarmonicPitchResult& out_result);

		const FixedVector<SnakeTrack, 64>& snakes() const { return snakes_; }

	  private:
		struct Peak
		{
			float freq = 0.0f;
			float amplitude = 0.0f;
		};

		static float hz_to_cents(float a, float b);
		static size_t find_nearest_band(const CochlearFrame& frame, float freq);
		static void center_snake_on_local_peak(const CochlearFrame& frame, SnakeTrack& snake);

		void detect_peaks(const CochlearFrame& frame, FixedVector<Peak, 128>& out_peaks) const;
		void update_snakes(const CochlearFrame& frame, const FixedVector<Peak, 128>& peaks);
		bool find_harmonic_set(HarmonicPitchResult& out_result) const;

	  private:
		SnakePitchTrackerConfig config_{};
		FixedVector<SnakeTrack, 64> snakes_;
	};

} // namespace robotick
