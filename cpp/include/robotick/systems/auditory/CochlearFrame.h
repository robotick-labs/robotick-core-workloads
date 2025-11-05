#pragma once

#include "robotick/api.h"
#include "robotick/systems/audio/AudioFrame.h"

namespace robotick
{
	struct CochlearFrame
	{
		// === Perceptual amplitude across frequency bands (|analytic signal|) ===
		// Each element corresponds to the instantaneous energy (envelope)
		// within a specific cochlear / ERB-spaced frequency channel.
		// - Range: typically 0.0–1.0 (after compression/normalisation)
		// - Captures loudness and temporal modulation cues
		// - e.g. Used by HarmonicPitchWorkload to detect onsets, envelopes, and harmonics
		AudioBuffer128 envelope;

		// === Instantaneous fine-grain phase across frequency bands (arg(analytic signal)) ===
		// The instantaneous phase angle of each band’s analytic (Hilbert) signal.
		// - Units: radians (–π..+π), continuous between successive frames
		// - Encodes microstructure timing of the original waveform (zero-crossings)
		// - Preserves the exact fine-temporal pattern needed for f₀ or waveform reconstruction
		// - e.g. Used by VoiceIsolatorWorkload for resynthesis via envelope * cos(phase)
		AudioBuffer128 fine_phase;

		// === Low-frequency (2–20 Hz) envelope modulation power per band ===
		// Measures rhythmic fluctuation energy in each band’s amplitude envelope.
		// - Captures prosodic or syllabic rhythm cues (speech rate, tremolo, etc.)
		// - Computed as short-term power of the band-envelope derivative or filtered energy
		// - e.g. Useful for HarmonicPitch and ProsodyAnalyser workloads
		AudioBuffer128 modulation_power;

		// === Absolute timestamp of this frame (seconds since boot) ===
		// Used to align auditory frames with other sensory modalities and for
		// temporal correlation across workloads.
		double timestamp = 0.0;

		// === Centre-frequency of each band ===
		AudioBuffer128 band_center_hz;
	};

} // namespace robotick
