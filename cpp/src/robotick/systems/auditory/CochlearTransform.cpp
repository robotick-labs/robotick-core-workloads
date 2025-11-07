// Copyright Robotick
// SPDX-License-Identifier: Apache-2.0
//
// CochlearTransform.cpp

#include "robotick/systems/auditory/CochlearTransform.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace robotick
{
	// ---------------- ERB helpers ----------------

	float CochlearTransform::erb_rate(float frequency_hz)
	{
		return 21.4f * std::log10(4.37e-3f * frequency_hz + 1.0f);
	}

	float CochlearTransform::inv_erb_rate(float erb_value)
	{
		return (std::pow(10.0f, erb_value / 21.4f) - 1.0f) / 4.37e-3f;
	}

	int CochlearTransform::clamp_fft_bin_index(int bin_index)
	{
		if (bin_index < 0)
		{
			return 0;
		}

		const int last_valid_bin = static_cast<int>(CochlearTransformState::fft_bins) - 1;

		if (bin_index > last_valid_bin)
		{
			return last_valid_bin;
		}

		return bin_index;
	}

	int CochlearTransform::hz_to_fft_bin(float frequency_hz, uint32_t sample_rate_hz)
	{
		const float bin_width_hz = static_cast<float>(sample_rate_hz) / static_cast<float>(CochlearTransformState::fft_size);
		const int raw_index = static_cast<int>(std::round(frequency_hz / bin_width_hz));
		return clamp_fft_bin_index(raw_index);
	}

	// ---------------- Window/FFT planning ----------------

	void CochlearTransform::build_window(CochlearTransformState& state)
	{
		state.stft_window.set_size(CochlearTransformState::frame_size);

		const float num_window_samples = static_cast<float>(CochlearTransformState::frame_size);
		double energy_accumulator = 0.0;

		for (size_t sample_index = 0; sample_index < CochlearTransformState::frame_size; ++sample_index)
		{
			// Hann window: w[n] = 0.5 * (1 - cos(2*pi*n/(N-1))).
			const float window_value =
				0.5f * (1.0f - std::cos(2.0f * static_cast<float>(M_PI) * static_cast<float>(sample_index) / (num_window_samples - 1.0f)));

			state.stft_window[sample_index] = window_value;
			energy_accumulator += static_cast<double>(window_value) * static_cast<double>(window_value);
		}

		state.window_rms = (energy_accumulator > 0.0)
							   ? static_cast<float>(std::sqrt(energy_accumulator / static_cast<double>(CochlearTransformState::frame_size)))
							   : 1.0f;
	}

	void CochlearTransform::plan_fft(CochlearTransformState& state)
	{
		state.fft_input_time_domain.set_size(CochlearTransformState::frame_size);
		state.fft_input_time_domain.fill(0.0f);

		size_t kiss_cfg_length_bytes = sizeof(state.kiss_cfg_mem);
		state.kiss_config_fftr = kiss_fftr_alloc(static_cast<int>(CochlearTransformState::fft_size), 0, state.kiss_cfg_mem, &kiss_cfg_length_bytes);

		if (!state.kiss_config_fftr)
		{
			// Fallback to heap allocation if the scratch buffer is too small.
			state.kiss_config_fftr = kiss_fftr_alloc(static_cast<int>(CochlearTransformState::fft_size), 0, nullptr, nullptr);
		}

		ROBOTICK_ASSERT(state.kiss_config_fftr && "kiss_fftr_alloc failed");

		state.fft_magnitude.set_size(CochlearTransformState::fft_bins);
		state.fft_phase.set_size(CochlearTransformState::fft_bins);
		state.fft_output_freq_domain.set_size(CochlearTransformState::fft_bins);
	}

	void CochlearTransform::build_erb_bands(const CochlearTransformConfig& config, CochlearTransformState& state)
	{
		state.bands.set_size(config.num_bands);

		const float erb_at_min = erb_rate(config.fmin_hz);
		const float erb_at_max = erb_rate(config.fmax_hz);

		const float erb_step = (config.num_bands > 1) ? ((erb_at_max - erb_at_min) / static_cast<float>(config.num_bands - 1)) : 0.0f;

		for (uint16_t band_index = 0; band_index < config.num_bands; ++band_index)
		{
			const float erb_value = erb_at_min + erb_step * static_cast<float>(band_index);
			const float center_frequency_hz = inv_erb_rate(erb_value);

			CochlearTransformState::BandInfo& band_info = state.bands[band_index];
			band_info.center_hz = center_frequency_hz;

			// Glasberg & Moore ERB formula scaled by config.erb_bandwidth_scale.
			const float erb_bandwidth_hz = config.erb_bandwidth_scale * 24.7f * (4.37e-3f * center_frequency_hz + 1.0f);

			const float left_frequency_hz = std::max(config.fmin_hz, center_frequency_hz - erb_bandwidth_hz);
			const float right_frequency_hz = std::min(config.fmax_hz, center_frequency_hz + erb_bandwidth_hz);

			band_info.left_bin = hz_to_fft_bin(left_frequency_hz, state.sample_rate);
			band_info.center_bin = hz_to_fft_bin(center_frequency_hz, state.sample_rate);
			band_info.right_bin = hz_to_fft_bin(right_frequency_hz, state.sample_rate);

			// Ensure at least one-bin width and a center within the span.
			if (band_info.right_bin <= band_info.left_bin)
			{
				const int min_exclusive_right = band_info.left_bin + 1;
				band_info.right_bin = std::min(static_cast<int>(CochlearTransformState::fft_bins), min_exclusive_right);
			}

			if (band_info.center_bin < band_info.left_bin || band_info.center_bin >= band_info.right_bin)
			{
				const int span = std::max(1, band_info.right_bin - band_info.left_bin);
				const int proposed_center = band_info.left_bin + span / 2;
				band_info.center_bin = std::clamp(proposed_center, band_info.left_bin, band_info.right_bin - 1);
			}
		}
	}

	void CochlearTransform::build_env_filters(const CochlearTransformConfig& config, CochlearTransformState& state)
	{
		ROBOTICK_ASSERT_MSG(state.frame_rate_hz > 0.0f, "state.frame_rate_hz should have been set by the calling code");

		const double frame_period_seconds = 1.0 / state.frame_rate_hz;

		// Envelope low-pass.
		const double envelope_cutoff_hz = std::clamp(static_cast<double>(config.envelope_lp_hz), 0.5, 60.0);
		const double envelope_tau_seconds = 1.0 / (2.0 * M_PI * envelope_cutoff_hz);
		state.envelope_alpha = static_cast<float>(1.0 - std::exp(-frame_period_seconds / envelope_tau_seconds));

		// Secondary slow smoothing.
		const double slow_cutoff_hz = std::clamp(static_cast<double>(config.envelope_temporal_smooth_hz), 0.1, 30.0);
		const double slow_tau_seconds = 1.0 / (2.0 * M_PI * slow_cutoff_hz);
		state.envelope_slow_alpha = static_cast<float>(1.0 - std::exp(-frame_period_seconds / slow_tau_seconds));

		// Modulation high-pass (on envelope).
		{
			const double hp_cutoff_hz = std::max(0.1, static_cast<double>(config.mod_low_hz));
			const double exp_term = std::exp(-2.0 * M_PI * hp_cutoff_hz / state.frame_rate_hz);

			state.mod_hp_a0 = static_cast<float>((1.0 + exp_term) * 0.5);
			state.mod_hp_b1 = static_cast<float>(exp_term);
			state.mod_hp_c1 = static_cast<float>(exp_term);
		}

		// Modulation low-pass (after HP).
		{
			const double lp_cutoff_hz = std::max(0.1, static_cast<double>(config.mod_high_hz));
			const double exp_term = std::exp(-2.0 * M_PI * lp_cutoff_hz / state.frame_rate_hz);

			state.mod_lp_a0 = static_cast<float>(1.0 - exp_term);
			state.mod_lp_b1 = static_cast<float>(exp_term);
			state.mod_lp_c1 = static_cast<float>(exp_term);
		}
	}

	void CochlearTransform::reset_state(CochlearTransformState& state)
	{
		state.ring_buffer.set_size(CochlearTransformState::frame_size);
		state.ring_buffer.fill(0.0f);

		state.ring_write_index = 0;
		state.ring_filled_count = 0;
		state.samples_since_last_frame = 0;

		state.previous_envelope_per_band.fill(0.0f);
		state.previous_envelope_slow_per_band.fill(0.0f);
		state.mod_hp_state_z1.fill(0.0f);
		state.mod_lp_state_z1.fill(0.0f);

		state.previous_input_sample = 0.0f;
		state.dc_tracker_state = 0.0f;
	}

	void CochlearTransform::push_samples(
		const float* source_samples, size_t num_samples, const CochlearTransformConfig& config, CochlearTransformState& state)
	{
		if (source_samples == nullptr || num_samples == 0)
		{
			return;
		}

		for (size_t sample_index = 0; sample_index < num_samples; ++sample_index)
		{
			float input_sample = source_samples[sample_index];

			// Slow DC tracker (one-pole LP), then remove DC.
			state.dc_tracker_state = state.dc_tracker_alpha * state.dc_tracker_state + (1.0f - state.dc_tracker_alpha) * input_sample;
			input_sample -= state.dc_tracker_state;

			// Optional preemphasis: y[n] = x[n] - preemph * x[n-1]
			if (config.use_preemphasis)
			{
				const float emphasized_sample = input_sample - state.previous_input_sample * config.preemph;
				state.previous_input_sample = input_sample;
				input_sample = emphasized_sample;
			}

			state.ring_buffer[state.ring_write_index] = input_sample;
			state.ring_write_index = (state.ring_write_index + 1) % CochlearTransformState::frame_size;

			if (state.ring_filled_count < CochlearTransformState::frame_size)
			{
				++state.ring_filled_count;
			}

			++state.samples_since_last_frame;
		}
	}

	bool CochlearTransform::make_frame_from_ring(CochlearTransformState& state)
	{
		if (state.ring_filled_count < CochlearTransformState::frame_size || state.samples_since_last_frame < CochlearTransformState::hop_size)
		{
			return false;
		}

		size_t ring_read_index = state.ring_write_index;

		for (size_t frame_sample_index = 0; frame_sample_index < CochlearTransformState::frame_size; ++frame_sample_index)
		{
			const float windowed_sample = (state.ring_buffer[ring_read_index] * state.stft_window[frame_sample_index]) / state.window_rms;

			state.fft_input_time_domain[frame_sample_index] = windowed_sample;

			ring_read_index = (ring_read_index + 1) % CochlearTransformState::frame_size;
		}

		state.samples_since_last_frame -= CochlearTransformState::hop_size;
		return true;
	}

	void CochlearTransform::analyze_one_frame(const CochlearTransformConfig& config, CochlearTransformState& state, CochlearFrame& out_frame)
	{
		// Real-FFT the prepared frame.
		kiss_fftr(state.kiss_config_fftr, state.fft_input_time_domain.data(), state.fft_output_freq_domain.data());

		// Complex → magnitude/phase.
		for (size_t bin_index = 0; bin_index < CochlearTransformState::fft_bins; ++bin_index)
		{
			const float real_part = state.fft_output_freq_domain[bin_index].r;
			const float imag_part = state.fft_output_freq_domain[bin_index].i;

			const float magnitude = std::sqrt(real_part * real_part + imag_part * imag_part);
			state.fft_magnitude[bin_index] = magnitude + 1e-12f;
			state.fft_phase[bin_index] = std::atan2(imag_part, real_part);
		}

		// Light 3-tap blur along frequency.
		for (size_t bin_index = 1; bin_index + 1 < CochlearTransformState::fft_bins; ++bin_index)
		{
			const float neighbor_left = state.fft_magnitude[bin_index - 1];
			const float center_value = state.fft_magnitude[bin_index];
			const float neighbor_right = state.fft_magnitude[bin_index + 1];

			state.fft_magnitude[bin_index] = (neighbor_left + 2.0f * center_value + neighbor_right) * 0.25f;
		}

		// Prepare outputs to correct band count (caller usually does this at load).
		out_frame.envelope.set_size(state.bands.size());
		out_frame.fine_phase.set_size(state.bands.size());
		out_frame.modulation_power.set_size(state.bands.size());
		out_frame.band_center_hz.set_size(state.bands.size());

		const float bin_width_hz = static_cast<float>(state.sample_rate) / static_cast<float>(CochlearTransformState::fft_size);

		// Accumulate energy per ERB band with Gaussian weighting across bins.
		for (size_t band_index = 0; band_index < state.bands.size(); ++band_index)
		{
			const CochlearTransformState::BandInfo& band_info = state.bands[band_index];

			const float center_frequency_hz = band_info.center_hz;
			const float erb_bandwidth_hz = config.erb_bandwidth_scale * 24.7f * (4.37e-3f * center_frequency_hz + 1.0f);

			float weighted_energy_accumulator = 0.0f;
			float weight_sum = 0.0f;

			for (int bin_index = band_info.left_bin; bin_index < band_info.right_bin; ++bin_index)
			{
				const float bin_frequency_hz = bin_width_hz * static_cast<float>(bin_index);
				const float gaussian_argument = (bin_frequency_hz - center_frequency_hz) / (0.5f * erb_bandwidth_hz);

				const float bin_weight = std::exp(-0.5f * gaussian_argument * gaussian_argument);
				const float magnitude = state.fft_magnitude[bin_index];

				weighted_energy_accumulator += bin_weight * (magnitude * magnitude);
				weight_sum += bin_weight;
			}

			if (weight_sum > 0.0f)
			{
				weighted_energy_accumulator /= weight_sum;
			}

			const float band_amplitude = std::sqrt(weighted_energy_accumulator);

			// First-stage envelope smoothing (single pole).
			const float previous_envelope = state.previous_envelope_per_band[band_index];
			const float smoothed_envelope = state.envelope_alpha * band_amplitude + (1.0f - state.envelope_alpha) * previous_envelope;
			state.previous_envelope_per_band[band_index] = smoothed_envelope;

			// Static compression.
			const float compressed_envelope = std::pow(std::max(smoothed_envelope, 0.0f) + 1e-9f, config.compression_gamma);

			// Envelope modulation band-pass.
			float high_pass_output = state.mod_hp_a0 * compressed_envelope + state.mod_hp_b1 * state.mod_hp_state_z1[band_index];
			high_pass_output = zap_denorm(high_pass_output);
			state.mod_hp_state_z1[band_index] = compressed_envelope - state.mod_hp_c1 * high_pass_output;

			float low_pass_output = state.mod_lp_a0 * high_pass_output + state.mod_lp_b1 * state.mod_lp_state_z1[band_index];
			low_pass_output = zap_denorm(low_pass_output);
			state.mod_lp_state_z1[band_index] = high_pass_output - state.mod_lp_c1 * low_pass_output;

			// Secondary slow smoothing (mainly for viz).
			const float previous_slow_envelope = state.previous_envelope_slow_per_band[band_index];
			const float slowly_smoothed_envelope =
				state.envelope_slow_alpha * compressed_envelope + (1.0f - state.envelope_slow_alpha) * previous_slow_envelope;
			state.previous_envelope_slow_per_band[band_index] = slowly_smoothed_envelope;

			// Outputs:
			out_frame.envelope[band_index] = slowly_smoothed_envelope;
			out_frame.modulation_power[band_index] = low_pass_output * low_pass_output;
			out_frame.fine_phase[band_index] = state.fft_phase[band_info.center_bin];
			out_frame.band_center_hz[band_index] = band_info.center_hz;
		}
	}

} // namespace robotick
