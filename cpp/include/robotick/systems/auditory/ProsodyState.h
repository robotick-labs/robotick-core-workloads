#pragma once

namespace robotick
{
	struct ProsodyState
	{
		// Core
		float rms = 0.0f;
		float zcr = 0.0f;	   // zero-crossing rate (per-sample fraction)
		float pitch_hz = 0.0f; // F0
		float pitch_slope_hz_per_s = 0.0f;
		bool voiced = false;

		// Spectral
		float spectral_energy_rms = 0.0f;
		float spectral_energy_ratio = 0.0f;
		float spectral_centroid_hz = 0.0f;
		float spectral_bandwidth_hz = 0.0f;
		float spectral_flatness = 0.0f;

		// Advanced
		float speaking_rate_sps = 0.0f; // syllables per second (stub)
		float jitter = 0.0f;			// cycle-to-cycle F0 variance (stub)
		float shimmer = 0.0f;			// cycle-to-cycle amp variance (stub)
		float harmonicity_hnr = 0.0f;	// harmonic-to-noise ratio (stub)
		float formant_f1_hz = 0.0f;		// (stub)
		float formant_f2_hz = 0.0f;		// (stub)
		float formant_f3_hz = 0.0f;		// (stub)
	};

} // namespace robotick
