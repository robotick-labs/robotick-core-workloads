#pragma once

namespace robotick
{
	struct ProsodyState
	{
		// --- Core ---
		float rms = 0.0f;				// loudness (captures voiced + unvoiced)
		bool is_voiced = false;			// true if harmonic pitch present
		float voiced_confidence = 0.0f; // based on harmonic energy

		// --- Temporal rhythm ---
		float speaking_rate_sps = 0.0f; // syllables or bursts per second

		// --- Pitch ---
		float pitch_hz = 0.0f;
		float pitch_slope_hz_per_s = 0.0f;

		// --- Harmonic quality ---
		float harmonicity_hnr_db = 0.0f; // harmonic-to-noise ratio
		float jitter = 0.0f;			 // short-term F0 variation
		float shimmer = 0.0f;			 // short-term amplitude variation

		// --- Timbre / brightness ---
		float spectral_brightness = 0.0f; // derived from harmonic amplitude slope

		// --- Harmonics-focused descriptors ---
		float h1_to_h2_db = 0.0f;			 // H2 vs H1 (captures “open vs breathy/tense”)
		float harmonic_tilt_db_per_h = 0.0f; // dB falloff per harmonic (simple linear fit)
		float even_odd_ratio = 1.0f;		 // (H2+H4+...) / (H1+H3+...)  (vowel colour)
		float harmonic_support_ratio = 0.0f; // fraction of harmonics above a relative threshold
		float centroid_ratio = 0.0f;		 // (energy-weighted harmonic index) / N  (where the stack “sits”)
		float formant1_ratio = 0.0f;		 // strongest broad peak index / N (very rough F1 proxy)
		float formant2_ratio = 0.0f;		 // second strongest broad peak index / N
	};

} // namespace robotick
