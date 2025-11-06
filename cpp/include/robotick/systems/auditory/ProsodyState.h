#pragma once

namespace robotick
{
	struct ProsodyState
	{
		// --- Core ---
		float rms = 0.0f;				// loudness (captures voiced + unvoiced)
		bool voiced = false;			// true if harmonic pitch present
		float voiced_confidence = 0.0f; // based on harmonic energy

		// --- Pitch ---
		float pitch_hz = 0.0f;
		float pitch_slope_hz_per_s = 0.0f;

		// --- Harmonic quality ---
		float harmonicity_hnr_db = 0.0f; // harmonic-to-noise ratio
		float jitter = 0.0f;			 // short-term F0 variation
		float shimmer = 0.0f;			 // short-term amplitude variation

		// --- Timbre / brightness ---
		float spectral_brightness = 0.0f; // derived from harmonic amplitude slope

		// --- Temporal rhythm ---
		float speaking_rate_sps = 0.0f; // syllables or bursts per second
	};

} // namespace robotick
