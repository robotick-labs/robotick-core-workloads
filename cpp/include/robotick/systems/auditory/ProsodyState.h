#pragma once

namespace robotick
{
	namespace Prosody
	{
		static constexpr int MaxPartials = 8; // harmonics above the fundamental

	}; // namespace Prosody

	using ProsodyPartials = FixedVector<float, Prosody::MaxPartials>;

	struct ProsodyState
	{
		// ===== Core =====
		float rms = 0.0f;
		float zcr = 0.0f;	   // zero-crossing rate (per-sample fraction)
		float pitch_hz = 0.0f; // F0
		float pitch_slope_hz_per_s = 0.0f;
		bool voiced = false;			// VAD gate
		float voiced_confidence = 0.0f; // optional [0..1] confidence (0 if unknown)

		// ===== Spectral summary =====
		float spectral_energy_rms = 0.0f;
		float spectral_energy_ratio = 0.0f; // spectral_rms / time_rms
		float spectral_centroid_hz = 0.0f;
		float spectral_bandwidth_hz = 0.0f; // spread (stdev) around centroid
		float spectral_flatness = 0.0f;		// geometric/arith mean ratio (0=peaky, 1=white-ish)
		float spectral_rolloff_hz = 0.0f;	// 85% energy roll-off (approx; 0 if unknown)
		float spectral_slope = 0.0f;		// coarse dB/dec slope (approx; 0 if unknown)

		// ===== Harmonicity / perturbation =====
		float harmonicity_hnr_db = 0.0f; // harmonic-to-noise ratio (dB; >0 more harmonic)
		float jitter = 0.0f;			 // F0 cycle jitter (stub-friendly)
		float shimmer = 0.0f;			 // amplitude cycle jitter (stub-friendly)

		// ===== Formants (optional; 0 if unknown) =====
		float formant_f1_hz = 0.0f;
		float formant_f2_hz = 0.0f;
		float formant_f3_hz = 0.0f;

		// ===== Partials from analyser (fixed-size, MCU friendly) =====
		int partial_count = 0;			 // 0..MaxPartials
		ProsodyPartials partial_gain;	 // linear gain per partial
		ProsodyPartials partial_freq_hz; // abs freq per partial
		bool partial_freq_valid = false; // if false, assume (h+2)*F0

		// ===== Speaking rate (coarse) =====
		float speaking_rate_sps = 0.0f; // syllables/sec (0 if unknown)
	};

} // namespace robotick
