// Copyright Robotick
// SPDX-License-Identifier: Apache-2.0
//
// CochlearTransform.h  (lean header: declarations only)

#pragma once

#include "robotick/api.h"
#include "robotick/framework/containers/FixedVector.h"
#include "robotick/framework/math/MathUtils.h"
#include "robotick/systems/audio/AudioFrame.h"
#include "robotick/systems/auditory/CochlearFrame.h"

#include <cstdint>
#include <math.h>
#include <kissfft/kiss_fftr.h>

namespace robotick
{
	struct CochlearTransformConfig
	{
		uint16_t num_bands = 128;

		// Frequency range (Hz) covered by the analysis.
		float fmin_hz = 50.0f;
		float fmax_hz = 3500.0f;

		// First-stage per-band envelope low-pass cutoff (Hz).
		float envelope_lp_hz = 100.0f;

		// Static dynamic-range compression (y = x^gamma). gamma < 1 compresses.
		float compression_gamma = 1.0f;

		// Modulation band-pass on the envelope (HP then LP), in Hz.
		float mod_low_hz = 1.0f;
		float mod_high_hz = 12.0f;

		// ERB width scale (dimensionless). Smaller => narrower bands.
		float erb_bandwidth_scale = 0.5f;

		// Optional input preemphasis (first-order high-pass-like).
		bool use_preemphasis = true;
		float preemph = 0.97f;

		// Secondary slow smoothing (Hz) over the compressed envelope (mainly for visualization).
		float envelope_temporal_smooth_hz = 5.0f;
	};

	// Plain state container (no methods).
	struct CochlearTransformState
	{
		// Frame and FFT geometry.
		static constexpr size_t frame_size = 4096;
		static constexpr size_t hop_size = frame_size / 4; // 75% overlap
		static constexpr size_t fft_size = frame_size;
		static constexpr size_t fft_bins = fft_size / 2 + 1;

		// One ERB band with its frequency-bin coverage.
		struct BandInfo
		{
			float center_hz = 0.0f;
			int left_bin = 0;
			int center_bin = 0;
			int right_bin = 0;
		};

		// Sample/frame rates.
		uint32_t sample_rate = 44100;
		double frame_rate_hz = 0.0; // sample_rate / hop_size

		// STFT buffers.
		FixedVector<float, frame_size> stft_window;
		FixedVector<float, frame_size> fft_input_time_domain;
		FixedVector<float, fft_bins> fft_magnitude;
		FixedVector<float, fft_bins> fft_phase;
		FixedVector<kiss_fft_cpx, fft_bins> fft_output_freq_domain;

		// Streaming ring buffer for overlap-add style framing.
		FixedVector<float, frame_size> ring_buffer;
		size_t ring_write_index = 0;
		size_t ring_filled_count = 0;
		size_t samples_since_last_frame = 0;

		// ERB bands.
		FixedVector<BandInfo, AudioBuffer128::capacity()> bands;

		// Envelope smoothing state.
		float envelope_alpha = 0.0f;
		AudioBuffer128 previous_envelope_per_band;

		// Envelope modulation filters (one-pole HP then LP) and state.
		float mod_hp_a0 = 0.0f, mod_hp_b1 = 0.0f, mod_hp_c1 = 0.0f;
		float mod_lp_a0 = 0.0f, mod_lp_b1 = 0.0f, mod_lp_c1 = 0.0f;
		AudioBuffer128 mod_hp_state_z1;
		AudioBuffer128 mod_lp_state_z1;

		// Preemphasis + DC removal.
		float previous_input_sample = 0.0f;
		float dc_tracker_state = 0.0f;
		float dc_tracker_alpha = 0.9995f;

		// kissFFT config + optional scratch memory.
		kiss_fftr_cfg kiss_config_fftr = nullptr;
		alignas(16) unsigned char kiss_cfg_mem[131072]{};

		// Hann window RMS (approximate energy preservation).
		float window_rms = 1.0f;

		// Secondary slow smoothing over the compressed envelope.
		float envelope_slow_alpha = 0.0f;
		AudioBuffer128 previous_envelope_slow_per_band;
	};

	class CochlearTransform
	{
	  public:
		// Build Hann window (tapers edges to reduce spectral leakage).
		static void build_window(CochlearTransformState& state);

		// Allocate/plan FFT and size working arrays.
		static void plan_fft(CochlearTransformState& state);

		// Build ERB-spaced bands and map them to FFT bin ranges.
		static void build_erb_bands(const CochlearTransformConfig& config, CochlearTransformState& state);

		// Precompute envelope smoothing + modulation filter coefficients.
		static void build_env_filters(const CochlearTransformConfig& config, CochlearTransformState& state);

		// Zero runtime state (ring buffer, filter memories, etc.).
		static void reset_state(CochlearTransformState& state);

		// Stream audio samples into the ring, with DC removal and optional preemphasis.
		static void push_samples(
			const float* source_samples, size_t num_samples, const CochlearTransformConfig& config, CochlearTransformState& state);

		// If enough samples are present, build the next windowed frame into fft_input_time_domain.
		static bool make_frame_from_ring(CochlearTransformState& state);

		// Perform one analysis step: STFT → per-band envelope → compression → modulation → outputs.
		static void analyze_one_frame(const CochlearTransformConfig& config, CochlearTransformState& state, CochlearFrame& out_frame);

		// ---------- Small helpers (exposed for unit tests) ----------
		static float erb_rate(float frequency_hz);	// ERB scale (Hz → ERB)
		static float inv_erb_rate(float erb_value); // inverse ERB (ERB → Hz)
		static int hz_to_fft_bin(float frequency_hz, uint32_t sample_rate_hz);
		static int clamp_fft_bin_index(int bin_index);

		// Denormal suppression for tiny floats.
		static inline float zap_denorm(float value) { return (fabsf(value) < 1e-30f) ? 0.0f : value; }
	};

} // namespace robotick
