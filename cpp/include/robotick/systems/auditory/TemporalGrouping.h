// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0
//
// TemporalGrouping.h  (lean header: declarations only)

#pragma once
#include <cstdint>

namespace robotick
{

	struct TemporalGroupingConfig
	{
		// Must match producer (cochlear)
		float fmin_hz = 50.0f;
		float fmax_hz = 3500.0f;
		uint16_t num_bands = 128;

		// f0 sweep
		float f0_min_hz = 60.0f;
		float f0_max_hz = 2000.0f;

		// Harmonic sieve
		uint8_t max_harmonics = 10;
		float harmonic_tolerance_cents = 35.0f;

		// Selection / gating
		float min_harmonicity = 0.15f;
		float min_amplitude = 0.3f;
		float reuse_penalty = 0.45f;

		// History usage
		uint8_t history_frames = 16;
		float coherence_min_window_s = 0.08f;

		// Modulation (Goertzel targets)
		uint8_t modulation_bins = 7; // {2,3,4,5,6,8,10}

		// Missing fundamental logic (OFF by default; typical speech keeps h1)
		bool infer_missing_fundamental = false;

		// Smoothing used by caller (EMA), not inside the lib
		float smooth_alpha = 0.5f;

		// maximum sources we should be aiming to detect
		uint8_t max_sources = 3;
	};

	struct TemporalGroupingResult
	{
		float f0_hz = 0.0f;
		float harmonicity = 0.0f;
		float amplitude = 0.0f;
		float centroid_hz = 0.0f;
		float bandwidth_hz = 0.0f;

		float temporal_coherence = 0.0f;
		float modulation_rate_hz = 0.0f;

		// contributing unique bands
		uint16_t bands[96];
		uint8_t band_count = 0;
	};

	class TemporalGrouping
	{
	  public:
		// Single-frame f0 evaluation with soft deconflict mask.
		// band_center_hz/envelope/claimed lengths = nb.
		// Writes a fully-populated result if candidate passes; otherwise band_count==0.
		static void eval_f0_with_mask(const float* band_center_hz,
			const float* envelope,
			const float* claimed, // may be nullptr (treated as zeros)
			int nb,
			const TemporalGroupingConfig& cfg,
			float f0,
			TemporalGroupingResult& out,
			float* E_h_out = nullptr // optional: energy per harmonic [32]
		);

		static int find_best_band_for_harmonic(float target_hz,
			const float* band_center_hz,
			const float* envelope,
			int num_bands,
			float tolerance_cents,
			float& out_within_tolerance,
			float& out_envelope);

		static float compute_band_contribution(float envelope, float within_tolerance, float claimed_fraction, const TemporalGroupingConfig& config);

		static bool passes_missing_fundamental_gate(const TemporalGroupingConfig& config,
			bool fundamental_hit,
			const float* harmonic_energy,
			uint8_t band_count,
			float early_energy_fraction,
			uint8_t early_hits);

		static void apply_span_based_harmonicity_adjustment(const float* band_center_hz, int num_bands, TemporalGroupingResult& out);

		// Temporal coherence over a band group using caller-provided history.
		// history_env: array of N pointers, each to an env frame of length nb.
		// timestamps: length N, ascending in time.
		// Returns 0..1 and sets out_group_env_mean.
		static float temporal_coherence_score(const float* const* history_env,
			const double* timestamps,
			uint8_t history_count,
			uint8_t history_cap,
			const uint16_t* band_indices,
			uint8_t band_count,
			int nb,
			float min_window_s,
			float& out_group_env_mean);

		// Modulation-rate estimate (Goertzel over group envelope).
		// Probes {2,3,4,5,6,8,10} Hz (limited by cfg.modulation_bins).
		static float estimate_modulation_rate_hz(const float* const* history_envelopes,
			uint8_t history_count,
			uint8_t /*history_cap*/,
			const uint16_t* selected_band_indices,
			uint8_t selected_band_count,
			int num_bands,
			float tick_rate_hz,
			const TemporalGroupingConfig& config);

		// Utility (kept public for tests)
		static int band_index_for_hz(const float* band_center_hz, int nb, float hz);
		static float band_local_width_hz(const float* band_center_hz, int nb, int j);
		static float band_width_cents(float* band_center_hz, int num_bands, int band_index);
		static float cents_between(float f1, float f2);
		static float clampf(float v, float lo, float hi);
	};

} // namespace robotick
