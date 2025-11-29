// Copyright Robotick
// SPDX-License-Identifier: Apache-2.0
//
// CochlearTransform.test.cpp

#include "robotick/systems/auditory/CochlearTransform.h"

#include "robotick/framework/containers/HeapVector.h"

#include "robotick/framework/math/Abs.h"
#include "robotick/framework/math/Trig.h"

#include <catch2/catch_all.hpp>

#include <cmath>

namespace robotick::test
{
	static float generate_sine_sample(float frequency_hz, float sample_rate, size_t sample_index)
	{
		const float two_pi = 2.0f * static_cast<float>(M_PI);
		return robotick::sin(two_pi * frequency_hz * (static_cast<float>(sample_index) / sample_rate));
	}

	static size_t index_of_max_value(const AudioBuffer128& buffer)
	{
		size_t best_index = 0;
		float best_value = -1e30f;

		for (size_t element_index = 0; element_index < buffer.size(); ++element_index)
		{
			const float value = buffer[element_index];
			if (value > best_value)
			{
				best_value = value;
				best_index = element_index;
			}
		}
		return best_index;
	}

	TEST_CASE("Unit/Audio/CochlearTransform")
	{
		const uint32_t sample_rate_hz = 44100;

		SECTION("Window RMS and FFT plan are sane")
		{
			// -----------------------------------------------------------------------------
			// TEST: Window RMS and FFT plan are sane
			//
			// Verifies that the analysis front-end is correctly initialised.
			// - The Hann window must have non-zero RMS (energy preservation).
			// - kissFFT plan allocation must succeed and produce expected bin count.
			//
			// This ensures later frequency-domain analyses will operate on properly
			// normalised, valid spectral data.
			// -----------------------------------------------------------------------------

			CochlearTransformState state;
			state.sample_rate = sample_rate_hz;

			CochlearTransform::build_window(state);
			CHECK(state.window_rms > 0.0f);

			CochlearTransform::plan_fft(state);
			REQUIRE(state.kiss_config_fftr != nullptr);
			CHECK(state.fft_magnitude.size() == CochlearTransformState::fft_bins);
		}

		SECTION("ERB band centers are monotonic and within range")
		{
			// -----------------------------------------------------------------------------
			// TEST: ERB band centers are monotonic and within range
			//
			// Confirms that ERB spacing produces strictly increasing band centres from
			// fmin_hz to fmax_hz, and that FFT bin indices for each band are valid and
			// ordered. Edge cases (first and final band) are allowed to touch DC or
			// Nyquist respectively.
			//
			// This validates the spectral geometry of the cochlear analysis front-end.
			// -----------------------------------------------------------------------------

			CochlearTransformConfig config;
			config.num_bands = 64;
			config.fmin_hz = 100.0f;
			config.fmax_hz = 8000.0f;

			CochlearTransformState state;
			state.sample_rate = sample_rate_hz;

			state.frame_rate_hz = static_cast<double>(sample_rate_hz) / static_cast<double>(CochlearTransformState::hop_size);

			CochlearTransform::build_erb_bands(config, state);

			REQUIRE(state.bands.size() == config.num_bands);
			float previous_center = 0.0f;

			for (size_t band_index = 0; band_index < state.bands.size(); ++band_index)
			{
				const float center_hz = state.bands[band_index].center_hz;
				CHECK(center_hz >= config.fmin_hz);
				CHECK(center_hz <= config.fmax_hz);
				if (band_index > 0)
				{
					CHECK(center_hz > previous_center);
				}
				previous_center = center_hz;

				// Bin indices should be ordered and valid.
				if (band_index == 0)
				{
					// Lowest band may have left == center (both at DC)
					CHECK(state.bands[band_index].left_bin <= state.bands[band_index].center_bin);
				}
				else
				{
					CHECK(state.bands[band_index].left_bin < state.bands[band_index].center_bin);
				}

				if (band_index + 1 < state.bands.size())
				{
					// For all but the final band, require strictly increasing bin order.
					CHECK(state.bands[band_index].center_bin < state.bands[band_index].right_bin);
				}
				else
				{
					// For the final band, allow center == right (edge of FFT range).
					CHECK(state.bands[band_index].center_bin <= state.bands[band_index].right_bin);
				}

				CHECK(state.bands[band_index].right_bin < static_cast<int>(CochlearTransformState::fft_bins));
			}
		}

		SECTION("Frame building requires enough samples and respects overlap")
		{
			// -----------------------------------------------------------------------------
			// TEST: Frame building requires enough samples and respects overlap
			//
			// Ensures that the ring-buffered frame construction logic behaves correctly:
			// - A frame cannot be built until at least one full frame_size of samples
			//   has been written to the buffer.
			// - Adding hop_size samples advances the window by one frame.
			//
			// This validates the temporal framing logic used for real-time STFT analysis.
			// -----------------------------------------------------------------------------

			CochlearTransformConfig config;

			CochlearTransformState state;
			state.sample_rate = sample_rate_hz;
			state.frame_rate_hz = static_cast<double>(sample_rate_hz) / static_cast<double>(CochlearTransformState::hop_size);

			CochlearTransform::build_window(state);
			CochlearTransform::plan_fft(state);
			CochlearTransform::build_erb_bands(config, state);
			CochlearTransform::build_env_filters(config, state);
			CochlearTransform::reset_state(state);

			bool have_frame = CochlearTransform::make_frame_from_ring(state);
			CHECK_FALSE(have_frame);

			float silence[CochlearTransformState::frame_size] = {};
			CochlearTransform::push_samples(silence, CochlearTransformState::frame_size, config, state);

			have_frame = CochlearTransform::make_frame_from_ring(state);
			CHECK(have_frame); // first frame now available

			float more_silence[CochlearTransformState::hop_size] = {};
			CochlearTransform::push_samples(more_silence, CochlearTransformState::hop_size, config, state);
			have_frame = CochlearTransform::make_frame_from_ring(state);
			CHECK(have_frame); // second frame available
		}

		SECTION("Single-tone sine produces maximal envelope near its band center")
		{
			// -----------------------------------------------------------------------------
			// TEST: Single-tone sine produces maximal envelope near its band center
			//
			// Feeds a pure sine tone into the full analysis pipeline and checks that:
			// - The highest envelope amplitude occurs in the ERB band whose centre
			//   frequency lies closest to the sine’s frequency.
			// - The resulting envelope is non-trivial, confirming correct spectral energy
			//   propagation through windowing, FFT, envelope detection and compression.
			//
			// This demonstrates that the cochlear transform behaves as a frequency-selective
			// filterbank whose output amplitudes correspond to perceived energy in each band.
			// -----------------------------------------------------------------------------

			CochlearTransformConfig config;
			config.num_bands = 96;
			config.fmin_hz = 80.0f;
			config.fmax_hz = 4000.0f;
			config.envelope_lp_hz = 80.0f; // reasonably quick envelope
			config.envelope_temporal_smooth_hz = 5.0f;

			CochlearTransformState state;
			state.sample_rate = sample_rate_hz;
			state.frame_rate_hz = static_cast<double>(sample_rate_hz) / static_cast<double>(CochlearTransformState::hop_size);

			CochlearTransform::build_window(state);
			CochlearTransform::plan_fft(state);
			CochlearTransform::build_erb_bands(config, state);
			CochlearTransform::build_env_filters(config, state);
			CochlearTransform::reset_state(state);

			const float target_tone_hz = 1200.0f;
			const size_t total_samples = CochlearTransformState::frame_size + CochlearTransformState::hop_size;

			float tone_buffer[CochlearTransformState::frame_size + CochlearTransformState::hop_size] = {};
			for (size_t sample_index = 0; sample_index < total_samples; ++sample_index)
			{
				tone_buffer[sample_index] = generate_sine_sample(target_tone_hz, static_cast<float>(sample_rate_hz), sample_index);
			}

			CochlearTransform::push_samples(tone_buffer, total_samples, config, state);

			REQUIRE(CochlearTransform::make_frame_from_ring(state));

			CochlearFrame frame_a;
			frame_a.envelope.set_size(config.num_bands);
			frame_a.fine_phase.set_size(config.num_bands);
			frame_a.modulation_power.set_size(config.num_bands);
			frame_a.band_center_hz.set_size(config.num_bands);

			CochlearTransform::analyze_one_frame(config, state, frame_a);

			REQUIRE(CochlearTransform::make_frame_from_ring(state));

			CochlearFrame frame_b;
			frame_b.envelope.set_size(config.num_bands);
			frame_b.fine_phase.set_size(config.num_bands);
			frame_b.modulation_power.set_size(config.num_bands);
			frame_b.band_center_hz.set_size(config.num_bands);

			CochlearTransform::analyze_one_frame(config, state, frame_b);

			const size_t max_band_index = index_of_max_value(frame_b.envelope);
			REQUIRE(max_band_index < frame_b.band_center_hz.size());

			const float detected_center_hz = frame_b.band_center_hz[max_band_index];

			CHECK(robotick::abs(detected_center_hz - target_tone_hz) < 100.0f);
			CHECK(frame_b.envelope[max_band_index] > 0.05f);
		}
	}

} // namespace robotick::test
