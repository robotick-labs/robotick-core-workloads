// Copyright Robotick
// SPDX-License-Identifier: Apache-2.0

#include "robotick/systems/auditory/TemporalGroupingV0.h"

#include <catch2/catch_all.hpp>

#include <cmath>
#include <cstring>
#include <vector>

// real-world data kept in seperate file for clarity
#include "TemporalGrouping_data.cpp"

namespace robotick::test
{
	static std::vector<float> make_linear_band_centers(float fmin, float fmax, int num_bands)
	{
		std::vector<float> v(num_bands);
		if (num_bands <= 1)
		{
			if (num_bands == 1)
				v[0] = fmin;
			return v;
		}
		const float step = (fmax - fmin) / float(num_bands - 1);
		for (int i = 0; i < num_bands; ++i)
			v[i] = fmin + step * float(i);
		return v;
	}

	static int argmin_abs(const std::vector<float>& xs, float f)
	{
		int best = -1;
		float bd = 1e30f;
		for (int i = 0; i < (int)xs.size(); ++i)
		{
			float d = std::fabs(xs[i] - f);
			if (d < bd)
			{
				bd = d;
				best = i;
			}
		}
		return best;
	}

	static float cents_to_hz(float base_hz, float cents)
	{
		return base_hz * (std::pow(2.0f, cents / 1200.0f) - 1.0f);
	}

	// Build N history frames of size num_bands with value function fn(frame_index, band_index)
	template <typename Fn>
	static void make_history(uint8_t num_history_entries,
		int num_bands,
		std::vector<std::vector<float>>& frames,
		std::vector<const float*>& frame_ptrs,
		std::vector<double>& timestamps,
		Fn fn,
		double t0 = 0.0,
		double dt = 1.0 / 80.0)
	{
		frames.assign(num_history_entries, std::vector<float>(num_bands, 0.0f));
		frame_ptrs.assign(num_history_entries, nullptr);
		timestamps.resize(num_history_entries);
		for (uint8_t history_entry_id = 0; history_entry_id < num_history_entries; ++history_entry_id)
		{
			for (int band_id = 0; band_id < num_bands; ++band_id)
			{
				frames[history_entry_id][band_id] = fn(history_entry_id, band_id);
			}

			frame_ptrs[history_entry_id] = frames[history_entry_id].data();
			timestamps[history_entry_id] = t0 + double(history_entry_id) * dt;
		}
	}

	// ---------- Tests ----------

	TEST_CASE("Unit/Audio/TemporalGroupingV0")
	{
		SECTION("Maps a given frequency to the closest ERB band index")
		{
			const int num_bands = 16;
			auto centers = make_linear_band_centers(100.0f, 2500.0f, num_bands);

			SECTION("Returns exact index for known center, and nearest for in-betweens")
			{
				// pick a center and check it maps to itself
				const int ix = 7;
				const float f = centers[ix];
				REQUIRE(TemporalGroupingV0::band_index_for_hz(centers.data(), num_bands, f) == ix);

				// halfway between ix and ix+1 should map to nearest (tie → higher index by our impl)
				const float mid = 0.5f * (centers[ix] + centers[ix + 1]);
				const int got = TemporalGroupingV0::band_index_for_hz(centers.data(), num_bands, mid + 1e-3f);
				REQUIRE(got == ix + 1);
			}
		}

		SECTION("Helpers for 'TemporalGroupingV0::eval_f0_with_mask' function as needed")
		{
			SECTION("find_best_band_for_harmonic selects correct neighbor within tolerance")
			{
				std::vector<float> centers = {1000.0f, 1100.0f, 1200.0f, 1300.0f};
				std::vector<float> envelope = {0.0f, 0.0f, 1.0f};

				float wt = 0.0f, amp = 0.0f;
				int ix = TemporalGroupingV0::find_best_band_for_harmonic(1200.0f, centers.data(), envelope.data(), 3, 35.0f, wt, amp);
				REQUIRE(ix == 2);
				CHECK(wt > 0.99f);
				CHECK(amp == 1.0f);
			}

			SECTION("compute_band_contribution scales by reuse and tolerance")
			{
				TemporalGroupingV0Settings config;
				config.reuse_penalty = 0.5f;

				const float envelope = 1.0f;
				const float tolerance = 0.8f;
				const float claimed = 0.4f;

				const float expected = envelope * tolerance * (1.0f - 0.5f * 0.4f);
				const float actual = TemporalGroupingV0::compute_band_contribution(envelope, tolerance, claimed, config);
				CHECK(actual == Catch::Approx(expected));
			}

			SECTION("passes_missing_fundamental_gate enforces early harmonic criteria")
			{
				TemporalGroupingV0Settings config;
				config.infer_missing_fundamental = true;

				float E[32] = {0.0f};
				E[2] = 0.6f;
				E[3] = 0.4f;

				bool pass = TemporalGroupingV0::passes_missing_fundamental_gate(config, false, E, 2, 0.5f, 2);
				CHECK(pass);

				pass = TemporalGroupingV0::passes_missing_fundamental_gate(config, false, E, 1, 0.5f, 1);
				CHECK_FALSE(pass);
			}
		}

		SECTION("Detects only the true fundamental and rejects all other f0 candidates")
		{
			SECTION("Rejects all f0 candidates except 1200 Hz")
			{
				TemporalGroupingV0Settings config;
				config.fmin_hz = 100.0f;
				config.fmax_hz = 3500.0f;
				config.num_bands = 64;
				config.f0_min_hz = 60.0f;
				config.f0_max_hz = 1400.0f;
				config.max_harmonics = 10;
				config.harmonic_tolerance_cents = 35.0f;
				config.min_harmonicity = 0.10f;
				config.min_amplitude = 0.001f;
				config.reuse_penalty = 0.45f;
				config.infer_missing_fundamental = false;

				const int nb = config.num_bands;
				auto centers = make_linear_band_centers(config.fmin_hz, config.fmax_hz, nb);
				std::vector<float> envelope(nb, 0.0f), claimed(nb, 0.0f);

				const int ix1200 = argmin_abs(centers, 1200.0f);
				REQUIRE(ix1200 >= 0);
				envelope[ix1200] = 1.0f;

				const float expected_f0 = 1200.0f;
				const float allowed_margin_hz = 2.0f * cents_to_hz(expected_f0, config.harmonic_tolerance_cents);

				// Sweep full f0 range excluding 1200 ± margin
				const float step_hz = 10.0f;
				for (float f0 = config.f0_min_hz; f0 <= config.f0_max_hz; f0 += step_hz)
				{
					if (std::fabs(f0 - 1200.0f) <= allowed_margin_hz)
						continue; // Skip the expected match

					TemporalGroupingV0Result r{};
					TemporalGroupingV0::eval_f0_with_mask(centers.data(), envelope.data(), claimed.data(), nb, config, f0, r, nullptr);

					CHECK(r.band_count == 0); // Everything else must be rejected
				}
			}

			SECTION("Correctly accepts 1200 Hz as f0")
			{
				TemporalGroupingV0Settings config;
				config.fmin_hz = 50.0f;
				config.fmax_hz = 3500.0f;
				config.num_bands = 64;
				config.f0_min_hz = 60.0f;
				config.f0_max_hz = 1400.0f;
				config.max_harmonics = 10;
				config.harmonic_tolerance_cents = 35.0f;
				config.min_harmonicity = 0.10f;
				config.min_amplitude = 0.001f;
				config.reuse_penalty = 0.45f;
				config.infer_missing_fundamental = false;

				const float expected_freq_hz = 1200.0f;

				const int nb = config.num_bands;
				auto centers = make_linear_band_centers(config.fmin_hz, config.fmax_hz, nb);
				std::vector<float> envelope(nb, 0.0f), claimed(nb, 0.0f);

				const int ix1200 = argmin_abs(centers, expected_freq_hz);
				REQUIRE(ix1200 >= 0);
				envelope[ix1200] = 1.0f;

				TemporalGroupingV0Result r{};
				TemporalGroupingV0::eval_f0_with_mask(centers.data(), envelope.data(), claimed.data(), nb, config, expected_freq_hz, r, nullptr);

				REQUIRE(r.band_count == 1);
				CHECK(r.f0_hz == Catch::Approx(expected_freq_hz).margin(5.0f));
				CHECK(r.centroid_hz == Catch::Approx(centers[ix1200]).margin(centers[1] - centers[0] + 1e-3f));
				CHECK(r.amplitude > 0.5f);
				CHECK(r.harmonicity > 0.5f);
			}
		}

		SECTION("Detects only the true fundamental in real-world envelope profile")
		{
			TemporalGroupingV0Settings config;
			config.fmin_hz = 50.0f;
			config.fmax_hz = 3500.0f;
			config.num_bands = 128;
			config.f0_min_hz = 60.0f;
			config.f0_max_hz = 1200.0f;
			config.max_harmonics = 10;
			config.harmonic_tolerance_cents = 35.0f;
			config.min_harmonicity = 0.15f;
			config.min_amplitude = 0.1f;
			config.reuse_penalty = 0.45f;
			config.infer_missing_fundamental = false;

			const int num_bands = config.num_bands;
			std::vector<float> envelope(num_bands, 0.0f), centers(num_bands, 0.0f), claimed(num_bands, 0.0f);

			// === Inject real-world envelope profile (128 values) ===

			static_assert(sizeof(s_real_1200hz_sinewave_centers) / sizeof(float) == 128, "Expected 128 values in real_centers");
			static_assert(sizeof(s_real_1200hz_sinewave_envelope) / sizeof(float) == 128, "Expected 128 values in real_profile");

			for (int i = 0; i < num_bands; ++i)
			{
				envelope[i] = s_real_1200hz_sinewave_envelope[i];
				centers[i] = s_real_1200hz_sinewave_centers[i];
			}

			// === Find closest band to 1200 Hz ===
			const int ix1200 = argmin_abs(centers, 1200.0f);
			REQUIRE(ix1200 >= 0);

			const float expected_f0 = 1200.0f;
			const float allowed_margin_hz = 2.0f * cents_to_hz(expected_f0, config.harmonic_tolerance_cents);

			SECTION("Rejects all f0 candidates except 1200 Hz")
			{
				const float step_hz = 10.0f;
				for (float f0 = config.f0_min_hz; f0 <= config.f0_max_hz; f0 += step_hz)
				{
					if (std::fabs(f0 - expected_f0) <= allowed_margin_hz)
						continue; // Skip expected match

					TemporalGroupingV0Result result{};
					TemporalGroupingV0::eval_f0_with_mask(centers.data(), envelope.data(), claimed.data(), num_bands, config, f0, result, nullptr);

					CHECK(result.band_count == 0); // All non-targets must be rejected
				}
			}

			SECTION("Correctly accepts 1200 Hz as f0")
			{
				TemporalGroupingV0Result r{};
				TemporalGroupingV0::eval_f0_with_mask(centers.data(), envelope.data(), claimed.data(), num_bands, config, 1200.f, r, nullptr);

				REQUIRE(r.band_count >= 1);
				CHECK(r.f0_hz == Catch::Approx(expected_f0).margin(5.0f));
				CHECK(r.centroid_hz >= 1100.0f);
				CHECK(r.amplitude > 0.005f);
				CHECK(r.harmonicity > 0.1f);
			}
		}

		SECTION("Infers missing fundamentals from strong harmonic pattern when enabled")
		{
			TemporalGroupingV0Settings config;
			config.fmin_hz = 100.0f;
			config.fmax_hz = 6000.0f;
			config.num_bands = 96;
			config.f0_min_hz = 60.0f;
			config.f0_max_hz = 2000.0f;
			config.max_harmonics = 10;
			config.harmonic_tolerance_cents = 35.0f;
			config.min_harmonicity = 0.10f;
			config.min_amplitude = 0.001f;
			config.reuse_penalty = 0.45f;

			const int num_bands = config.num_bands;
			auto centers = make_linear_band_centers(config.fmin_hz, config.fmax_hz, num_bands);
			std::vector<float> envelope(num_bands, 0.0f), claimed(num_bands, 0.0f);

			// Missing fundamental at 1200 Hz, but present h2=2400, h3=3600
			const int ix2400 = argmin_abs(centers, 2400.0f);
			const int ix3600 = argmin_abs(centers, 3600.0f);
			REQUIRE(ix2400 >= 0);
			REQUIRE(ix3600 >= 0);
			envelope[ix2400] = 1.0f;
			envelope[ix3600] = 0.8f;

			SECTION("Skips candidate if fundamental is missing and inference is disabled")
			{
				config.infer_missing_fundamental = false;
				TemporalGroupingV0Result r{};
				TemporalGroupingV0::eval_f0_with_mask(centers.data(), envelope.data(), claimed.data(), num_bands, config, 1200.0f, r, nullptr);
				CHECK(r.band_count == 0);
			}

			SECTION("Infers and accepts f0 if strong h2 and h3 are detected with inference enabled")
			{
				config.infer_missing_fundamental = true;
				TemporalGroupingV0Result r{};
				TemporalGroupingV0::eval_f0_with_mask(centers.data(), envelope.data(), claimed.data(), num_bands, config, 1200.0f, r, nullptr);

				REQUIRE(r.band_count >= 2);
				CHECK(r.f0_hz == Catch::Approx(1200.0f).margin(5.0f));
				CHECK(r.harmonicity > 0.2f);
				CHECK(r.amplitude > 0.3f);
			}
		}

		SECTION("Detects temporal coherence and estimates modulation rate across bands")
		{
			TemporalGroupingV0Settings config;
			config.history_frames = 16;
			config.coherence_min_window_s = 0.08f; // ensure enough time span for N=16 @ 80 Hz
			config.modulation_bins = 7;

			// Build tiny bank with two bands; both correlate in time
			const int num_bands = 8;
			std::vector<float> centers(num_bands, 0.0f);
			for (int j = 0; j < num_bands; ++j)
				centers[j] = 200.0f + 50.0f * float(j);

			const int b0 = 3, b1 = 4;
			const uint16_t group[2] = {(uint16_t)b0, (uint16_t)b1};

			// History: y(t) = 0.5 + 0.4 * sin(2*pi*4Hz * t)
			// We’ll sample at 80 Hz for N=16 frames → exactly 0.2 s
			const uint8_t num_history_entries = 16;
			std::vector<std::vector<float>> frames;
			std::vector<const float*> frame_ptrs;
			std::vector<double> timestamps;

			const double tick_rate_hz = 80.0;

			const double dt = 1.0 / tick_rate_hz;
			const double f_modulation = 4.0; // 4 Hz
			make_history(
				num_history_entries,
				num_bands,
				frames,
				frame_ptrs,
				timestamps,
				[&](uint8_t history_entry_id, int band_id) -> float
				{
					const double t = double(history_entry_id) * dt;
					const float y = 0.5f + 0.4f * std::sin(float(2.0 * M_PI * f_modulation * t));
					if (band_id == b0)
						return y;
					if (band_id == b1)
						return 0.8f * y; // correlated, scaled
					return 0.0f;
				},
				/*t0=*/0.0,
				dt);

			SECTION("Returns high coherence score for bands with similar temporal envelope")
			{
				// i.e. two bands are temporaily coherent if they rise and fall roughly together

				float group_mean = 0.0f;
				const float coh = TemporalGroupingV0::temporal_coherence_score(frame_ptrs.data(),
					timestamps.data(),
					num_history_entries,
					num_history_entries,
					group,
					2,
					num_bands,
					config.coherence_min_window_s,
					group_mean);
				REQUIRE(coh >= 0.0f);
				REQUIRE(coh <= 1.0f);
				CHECK(coh > 0.8f);
				CHECK(group_mean > 0.1f);
			}

			SECTION("Accurately estimates shared modulation frequency of grouped bands")
			{
				const float est = TemporalGroupingV0::estimate_modulation_rate_hz(
					frame_ptrs.data(), num_history_entries, num_history_entries, group, 2, num_bands, (float)tick_rate_hz, config);
				CHECK(est == Catch::Approx(f_modulation).margin(0.25f));
			}
		}

		SECTION("Reduces confidence when spectral energy is already claimed by another source")
		{
			TemporalGroupingV0Settings config;
			config.fmin_hz = 50.0f;
			config.fmax_hz = 3500.0f;
			config.num_bands = 64;
			config.f0_min_hz = 60.0f;
			config.f0_max_hz = 1400.0f;
			config.harmonic_tolerance_cents = 35.0f;
			config.reuse_penalty = 0.6f;

			const int num_bands = config.num_bands;
			auto centers = make_linear_band_centers(config.fmin_hz, config.fmax_hz, num_bands);
			std::vector<float> envelope(num_bands, 0.0f), claimed(num_bands, 0.0f);

			// Single strong ridge near 1200; mark it as claimed already.
			const int index_1200hz = argmin_abs(centers, 1200.0f);
			REQUIRE(index_1200hz >= 0);
			envelope[index_1200hz] = 1.0f;
			claimed[index_1200hz] = 1.0f; // heavily claimed

			TemporalGroupingV0Result r{};
			TemporalGroupingV0::eval_f0_with_mask(centers.data(), envelope.data(), claimed.data(), num_bands, config, 1200.0f, r, nullptr);

			// With heavy claim and reuse penalty, accepted amplitude/harmonicity should drop
			// (Exact thresholds depend on bin spacing; just assert they’re small but nonzero)
			CHECK(r.band_count >= 1);
			CHECK(r.harmonicity < 0.9f);
			CHECK(r.amplitude < 0.9f);
		}
	}

}; // namespace robotick::test
