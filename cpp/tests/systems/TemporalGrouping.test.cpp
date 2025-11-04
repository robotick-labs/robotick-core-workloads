// Copyright Robotick
// SPDX-License-Identifier: Apache-2.0

#include "robotick/systems/auditory/TemporalGrouping.h"

#include <catch2/catch_all.hpp>

#include <cmath>
#include <cstring>
#include <vector>

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

	TEST_CASE("Unit/Audio/TemporalGrouping")
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
				REQUIRE(TemporalGrouping::band_index_for_hz(centers.data(), num_bands, f) == ix);

				// halfway between ix and ix+1 should map to nearest (tie → higher index by our impl)
				const float mid = 0.5f * (centers[ix] + centers[ix + 1]);
				const int got = TemporalGrouping::band_index_for_hz(centers.data(), num_bands, mid + 1e-3f);
				REQUIRE(got == ix + 1);
			}
		}

		SECTION("Helpers for 'TemporalGrouping::eval_f0_with_mask' function as needed")
		{
			SECTION("find_best_band_for_harmonic selects correct neighbor within tolerance")
			{
				std::vector<float> centers = {1000.0f, 1100.0f, 1200.0f, 1300.0f};
				std::vector<float> envelope = {0.0f, 0.0f, 1.0f};

				float wt = 0.0f, amp = 0.0f;
				int ix = TemporalGrouping::find_best_band_for_harmonic(1200.0f, centers.data(), envelope.data(), 3, 35.0f, wt, amp);
				REQUIRE(ix == 2);
				CHECK(wt > 0.99f);
				CHECK(amp == 1.0f);
			}

			SECTION("compute_band_contribution scales by reuse and tolerance")
			{
				TemporalGroupingConfig cfg;
				cfg.reuse_penalty = 0.5f;

				const float envelope = 1.0f;
				const float tolerance = 0.8f;
				const float claimed = 0.4f;

				const float expected = envelope * tolerance * (1.0f - 0.5f * 0.4f);
				const float actual = TemporalGrouping::compute_band_contribution(envelope, tolerance, claimed, cfg);
				CHECK(actual == Catch::Approx(expected));
			}

			SECTION("passes_missing_fundamental_gate enforces early harmonic criteria")
			{
				TemporalGroupingConfig cfg;
				cfg.infer_missing_fundamental = true;

				float E[32] = {0.0f};
				E[2] = 0.6f;
				E[3] = 0.4f;

				bool pass = TemporalGrouping::passes_missing_fundamental_gate(cfg, false, E, 2, 0.5f, 2);
				CHECK(pass);

				pass = TemporalGrouping::passes_missing_fundamental_gate(cfg, false, E, 1, 0.5f, 1);
				CHECK_FALSE(pass);
			}
		}

		SECTION("Detects only the true fundamental and rejects all other f0 candidates")
		{
			SECTION("Rejects all f0 candidates except 1200 Hz")
			{
				TemporalGroupingConfig cfg;
				cfg.fmin_hz = 100.0f;
				cfg.fmax_hz = 3500.0f;
				cfg.num_bands = 64;
				cfg.f0_min_hz = 60.0f;
				cfg.f0_max_hz = 1400.0f;
				cfg.max_harmonics = 10;
				cfg.harmonic_tolerance_cents = 35.0f;
				cfg.min_harmonicity = 0.10f;
				cfg.min_amplitude = 0.001f;
				cfg.reuse_penalty = 0.45f;
				cfg.infer_missing_fundamental = false;

				const int nb = cfg.num_bands;
				auto centers = make_linear_band_centers(cfg.fmin_hz, cfg.fmax_hz, nb);
				std::vector<float> envelope(nb, 0.0f), claimed(nb, 0.0f);

				const int ix1200 = argmin_abs(centers, 1200.0f);
				REQUIRE(ix1200 >= 0);
				envelope[ix1200] = 1.0f;

				const float expected_f0 = 1200.0f;
				const float allowed_margin_hz = 2.0f * cents_to_hz(expected_f0, cfg.harmonic_tolerance_cents);

				// Sweep full f0 range excluding 1200 ± margin
				const float step_hz = 10.0f;
				for (float f0 = cfg.f0_min_hz; f0 <= cfg.f0_max_hz; f0 += step_hz)
				{
					if (std::fabs(f0 - 1200.0f) <= allowed_margin_hz)
						continue; // Skip the expected match

					TemporalGroupingResult r{};
					TemporalGrouping::eval_f0_with_mask(centers.data(), envelope.data(), claimed.data(), nb, cfg, f0, r, nullptr);

					CHECK(r.band_count == 0); // Everything else must be rejected
				}
			}

			SECTION("Correctly accepts 1200 Hz as f0")
			{
				TemporalGroupingConfig cfg;
				cfg.fmin_hz = 100.0f;
				cfg.fmax_hz = 3500.0f;
				cfg.num_bands = 64;
				cfg.f0_min_hz = 60.0f;
				cfg.f0_max_hz = 1400.0f;
				cfg.max_harmonics = 10;
				cfg.harmonic_tolerance_cents = 35.0f;
				cfg.min_harmonicity = 0.10f;
				cfg.min_amplitude = 0.001f;
				cfg.reuse_penalty = 0.45f;
				cfg.infer_missing_fundamental = false;

				const int nb = cfg.num_bands;
				auto centers = make_linear_band_centers(cfg.fmin_hz, cfg.fmax_hz, nb);
				std::vector<float> envelope(nb, 0.0f), claimed(nb, 0.0f);

				const int ix1200 = argmin_abs(centers, 1200.0f);
				REQUIRE(ix1200 >= 0);
				envelope[ix1200] = 1.0f;

				TemporalGroupingResult r{};
				TemporalGrouping::eval_f0_with_mask(centers.data(), envelope.data(), claimed.data(), nb, cfg, 1200.0f, r, nullptr);

				REQUIRE(r.band_count == 1);
				CHECK(r.f0_hz == Catch::Approx(1200.0f).margin(5.0f));
				CHECK(r.centroid_hz == Catch::Approx(centers[ix1200]).margin(centers[1] - centers[0] + 1e-3f));
				CHECK(r.amplitude == Catch::Approx(1.0f).margin(0.001f));
				CHECK(r.harmonicity > 0.5f);
			}
		}

		SECTION("Infers missing fundamentals from strong harmonic pattern when enabled")
		{
			TemporalGroupingConfig cfg;
			cfg.fmin_hz = 100.0f;
			cfg.fmax_hz = 6000.0f;
			cfg.num_bands = 96;
			cfg.f0_min_hz = 60.0f;
			cfg.f0_max_hz = 2000.0f;
			cfg.max_harmonics = 10;
			cfg.harmonic_tolerance_cents = 35.0f;
			cfg.min_harmonicity = 0.10f;
			cfg.min_amplitude = 0.001f;
			cfg.reuse_penalty = 0.45f;

			const int num_bands = cfg.num_bands;
			auto centers = make_linear_band_centers(cfg.fmin_hz, cfg.fmax_hz, num_bands);
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
				cfg.infer_missing_fundamental = false;
				TemporalGroupingResult r{};
				TemporalGrouping::eval_f0_with_mask(centers.data(), envelope.data(), claimed.data(), num_bands, cfg, 1200.0f, r, nullptr);
				CHECK(r.band_count == 0);
			}

			SECTION("Infers and accepts f0 if strong h2 and h3 are detected with inference enabled")
			{
				cfg.infer_missing_fundamental = true;
				TemporalGroupingResult r{};
				TemporalGrouping::eval_f0_with_mask(centers.data(), envelope.data(), claimed.data(), num_bands, cfg, 1200.0f, r, nullptr);

				REQUIRE(r.band_count >= 2);
				CHECK(r.f0_hz == Catch::Approx(1200.0f).margin(5.0f));
				CHECK(r.harmonicity > 0.2f);
				CHECK(r.amplitude > 0.3f);
			}
		}

		SECTION("Detects temporal coherence and estimates modulation rate across bands")
		{
			TemporalGroupingConfig cfg;
			cfg.history_frames = 16;
			cfg.coherence_min_window_s = 0.08f; // ensure enough time span for N=16 @ 80 Hz
			cfg.modulation_bins = 7;

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
				const float coh = TemporalGrouping::temporal_coherence_score(frame_ptrs.data(),
					timestamps.data(),
					num_history_entries,
					num_history_entries,
					group,
					2,
					num_bands,
					cfg.coherence_min_window_s,
					group_mean);
				REQUIRE(coh >= 0.0f);
				REQUIRE(coh <= 1.0f);
				CHECK(coh > 0.8f);
				CHECK(group_mean > 0.1f);
			}

			SECTION("Accurately estimates shared modulation frequency of grouped bands")
			{
				const float est = TemporalGrouping::estimate_modulation_rate_hz(
					frame_ptrs.data(), num_history_entries, num_history_entries, group, 2, num_bands, (float)tick_rate_hz, cfg);
				CHECK(est == Catch::Approx(f_modulation).margin(0.25f));
			}
		}

		SECTION("Reduces confidence when spectral energy is already claimed by another source")
		{
			TemporalGroupingConfig cfg;
			cfg.fmin_hz = 100.0f;
			cfg.fmax_hz = 3500.0f;
			cfg.num_bands = 64;
			cfg.f0_min_hz = 60.0f;
			cfg.f0_max_hz = 1400.0f;
			cfg.harmonic_tolerance_cents = 35.0f;
			cfg.reuse_penalty = 0.6f;

			const int num_bands = cfg.num_bands;
			auto centers = make_linear_band_centers(cfg.fmin_hz, cfg.fmax_hz, num_bands);
			std::vector<float> envelope(num_bands, 0.0f), claimed(num_bands, 0.0f);

			// Single strong ridge near 1200; mark it as claimed already.
			const int index_1200hz = argmin_abs(centers, 1200.0f);
			REQUIRE(index_1200hz >= 0);
			envelope[index_1200hz] = 1.0f;
			claimed[index_1200hz] = 1.0f; // heavily claimed

			TemporalGroupingResult r{};
			TemporalGrouping::eval_f0_with_mask(centers.data(), envelope.data(), claimed.data(), num_bands, cfg, 1200.0f, r, nullptr);

			// With heavy claim and reuse penalty, accepted amplitude/harmonicity should drop
			// (Exact thresholds depend on bin spacing; just assert they’re small but nonzero)
			CHECK(r.band_count >= 1);
			CHECK(r.harmonicity < 0.6f);
			CHECK(r.amplitude < 0.8f);
		}
	}

}; // namespace robotick::test
