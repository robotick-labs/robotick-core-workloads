// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0
//
// HarmonicPitch.h  (lean header: declarations only)

#pragma once

#include "robotick/api.h"
#include "robotick/framework/containers/FixedVector.h"
#include "robotick/systems/audio/AudioFrame.h"

#include <cstdint>

namespace robotick
{
	namespace harmonic_pitch
	{
		static constexpr size_t MaxHarmonics = 16;
	};

	struct HarmonicPitchSettings
	{
		// Selection / gating
		float min_amplitude = 0.05f;			// minimum envelope value for it to be considered an interesting feature
		float min_peak_falloff_norm = 0.1f;		// minimum falloff from a peak (as a fraction of its (peak-value - min_amplitude))
												// 	for it to count as a peak
		float harmonic_tolerance_cents = 50.0f; // i.e. hormonic's peak must be within 50% of a semitone to count as a matching harmonic
		bool allow_single_peak_mode = true;

		float min_total_continuation_amplitude = 1.0f;
	};

	using HarmonicAmplitudes = FixedVector<float, harmonic_pitch::MaxHarmonics>;

	struct HarmonicPitchResult
	{
		float h1_f0_hz = 0.0f;					// Detected fundamental frequency (Hz)
		HarmonicAmplitudes harmonic_amplitudes; // Raw amplitudes for h1, h2, ... up to MaxHarmonics

		float get_h1_amplitude() const { return harmonic_amplitudes.size() > 0 ? harmonic_amplitudes[0] : 0.0f; }
	};

	class HarmonicPitch
	{
	  public:
		// Analyses a cochlear envelope and attempts to detect a harmonic source.
		// This performs a fresh analysis of the current envelope only, with no knowledge of prior frames.
		//
		// Arguments:
		// - settings: pitch detection parameters (tolerances, max harmonics, etc)
		// - centers: band center frequencies of the cochlear frame
		// - envelope: amplitude per band from the cochlear transform
		// - result: will be populated with the best-matching f0 and harmonics if found
		//
		// Returns:
		// - true if a plausible harmonic source (f0 and harmonics) was detected
		// - false if no acceptable match was found
		static bool find_harmonic_features(
			const HarmonicPitchSettings& settings, const AudioBuffer128& centers, const AudioBuffer128& envelope, HarmonicPitchResult& result);

		// Attempts to continue a previously detected f0 from the last frame,
		// even if no new harmonic stack is detected.
		//
		// This allows a previously stable pitch to persist despite weak current-frame evidence,
		// provided that the nearby amplitude is still consistent and the frequency hasn't shifted too far.
		//
		// Arguments:
		// - settings: pitch continuation parameters (hold thresholds, tolerance)
		// - centers: band center frequencies
		// - envelope: amplitude per band
		// - prev_result: harmonic result from the previous frame (must be non-zero to be considered)
		// - result: will be populated with updated values if continuation is valid
		//
		// Returns:
		// - true if the previous result was successfully relocated and considered valid in this frame
		// - false if it cannot be continued (e.g. amplitude too low or frequency shift too large)
		static bool try_continue_previous_result(const HarmonicPitchSettings& settings,
			const AudioBuffer128& centers,
			const AudioBuffer128& envelope,
			const HarmonicPitchResult& prev_result,
			HarmonicPitchResult& result);

		// Attempts to detect a harmonic source in the current envelope, using either:
		// (1) a fresh detection from scratch, or
		// (2) continuation of a previously tracked f0 from the previous frame.
		//
		// It compares both options and selects whichever yields the strongest consistent harmonic structure.
		//
		// Arguments:
		// - settings: pitch detection parameters
		// - centers: band center frequencies of the cochlear frame
		// - envelope: amplitude per band from the cochlear transform
		// - prev_result: harmonic result from the previous frame (may be empty or zero)
		// - out_result: populated with the chosen harmonic structure for this frame
		//
		// Returns:
		// - true if a plausible harmonic structure (either fresh or continued) was found
		// - false if no valid f0 could be determined or continued
		static bool find_or_continue_harmonic_features(const HarmonicPitchSettings& settings,
			const AudioBuffer128& centers,
			const AudioBuffer128& envelope,
			const HarmonicPitchResult& prev_result,
			HarmonicPitchResult& out_result);
	};

} // namespace robotick
