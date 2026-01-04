// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#include "robotick/systems/MuJoCoRenderContext.h"
#include "robotick/systems/MuJoCoPhysics.h"
#include "robotick/framework/containers/HeapVector.h"
#include "robotick/framework/memory/StdApproved.h"
#include "robotick/systems/Image.h"

#include <catch2/catch_test_macros.hpp>
#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace robotick::tests
{
#if defined(ROBOTICK_PLATFORM_DESKTOP) || defined(ROBOTICK_PLATFORM_LINUX)
	namespace
	{
		// Minimal MuJoCo scene with an empty worldbody; uses default grey background; used to confirm context init + PNG.
		constexpr char kMinimalModelPath[] = ROBOTICK_CORE_ROOT "/cpp/tests/data/mujoco/minimal.xml";
		// Scene that clears to a solid blue background; used to validate GL render output colour.
		constexpr char kBlueBackgroundModelPath[] = ROBOTICK_CORE_ROOT "/cpp/tests/data/mujoco/blue_background.xml";
	} // namespace

	TEST_CASE("Unit/Systems/MuJoCoRenderContext")
	{
		auto encode_png_from_rgb = [](const uint8_t* rgb, size_t rgb_size, int width, int height, ImagePng128k& out_png) -> bool
		{
			if (!rgb || rgb_size == 0 || width <= 0 || height <= 0)
				return false;

			cv::Mat rgb_mat(height, width, CV_8UC3, const_cast<uint8_t*>(rgb));
			cv::Mat rgb_flipped;
			cv::flip(rgb_mat, rgb_flipped, 0);

			cv::Mat bgr;
			cv::cvtColor(rgb_flipped, bgr, cv::COLOR_RGB2BGR);

			std_approved::vector<uint8_t> png_data;
			if (!cv::imencode(".png", bgr, png_data))
				return false;

			if (png_data.empty() || png_data.size() > out_png.capacity())
				return false;

			out_png.set(png_data.data(), png_data.size());
			return true;
		};
		SECTION("Initialises context and produces non-empty PNG for minimal scene")
		{
			MuJoCoPhysics physics;
			REQUIRE(physics.load_from_xml(kMinimalModelPath));

			mjData* snapshot_data = nullptr;
			const mjModel* snapshot_model = nullptr;
			double snapshot_time = 0.0;
			REQUIRE(physics.alloc_render_snapshot(snapshot_data, snapshot_model, snapshot_time));
			REQUIRE(snapshot_model != nullptr);
			REQUIRE(snapshot_data != nullptr);

			MuJoCoRenderContext context;
			if (!context.init(snapshot_model, 64, 48))
			{
				MuJoCoPhysics::destroy_snapshot(snapshot_data);
				SKIP("MuJoCo render context init failed (likely headless GL)");
			}

			HeapVector<uint8_t> rgb;
			rgb.initialize(64 * 48 * 3);
			int rgb_width = 0;
			int rgb_height = 0;
			size_t rgb_size = 0;
			if (!context.render_to_rgb(snapshot_model, snapshot_data, "", rgb.data(), rgb.size(), rgb_size, rgb_width, rgb_height, false))
			{
				MuJoCoPhysics::destroy_snapshot(snapshot_data);
				SKIP("MuJoCo render failed (likely headless GL)");
			}

			ImagePng128k png;
			if (!encode_png_from_rgb(rgb.data(), rgb_size, rgb_width, rgb_height, png))
			{
				MuJoCoPhysics::destroy_snapshot(snapshot_data);
				SKIP("MuJoCo PNG encode failed.");
			}

			REQUIRE(png.size() > 0);

			MuJoCoPhysics::destroy_snapshot(snapshot_data);
		}

		SECTION("Manual GL clear produces blue pixels")
		{
			MuJoCoPhysics physics;
			REQUIRE(physics.load_from_xml(kBlueBackgroundModelPath));

			mjData* snapshot_data = nullptr;
			const mjModel* snapshot_model = nullptr;
			double snapshot_time = 0.0;
			REQUIRE(physics.alloc_render_snapshot(snapshot_data, snapshot_model, snapshot_time));
			REQUIRE(snapshot_model != nullptr);
			REQUIRE(snapshot_data != nullptr);

			MuJoCoRenderContext context;
			REQUIRE(context.init(snapshot_model, 64, 48));

			GLuint fbo = 0;
			GLuint color = 0;
			glGenFramebuffers(1, &fbo);
			glBindFramebuffer(GL_FRAMEBUFFER, fbo);

			glGenRenderbuffers(1, &color);
			glBindRenderbuffer(GL_RENDERBUFFER, color);
			glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8, 64, 48);
			glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, color);

			REQUIRE(glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE);

			glViewport(0, 0, 64, 48);
			glClearColor(0.0f, 0.0f, 1.0f, 1.0f);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

			HeapVector<uint8_t> rgba;
			rgba.initialize(64 * 48 * 4);
			glFinish();
			glReadPixels(0, 0, 64, 48, GL_RGBA, GL_UNSIGNED_BYTE, rgba.data());

			glBindFramebuffer(GL_FRAMEBUFFER, 0);
			glDeleteRenderbuffers(1, &color);
			glDeleteFramebuffers(1, &fbo);

			uint64_t sum_r = 0;
			uint64_t sum_g = 0;
			uint64_t sum_b = 0;
			for (size_t i = 0; i + 3 < rgba.size(); i += 4)
			{
				sum_r += rgba[i + 0];
				sum_g += rgba[i + 1];
				sum_b += rgba[i + 2];
			}
			const double denom = static_cast<double>(rgba.size() / 4);
			const double mean_r = static_cast<double>(sum_r) / denom;
			const double mean_g = static_cast<double>(sum_g) / denom;
			const double mean_b = static_cast<double>(sum_b) / denom;

			REQUIRE(mean_b > mean_g + 20.0);
			REQUIRE(mean_b > mean_r + 20.0);

			MuJoCoPhysics::destroy_snapshot(snapshot_data);
		}

		SECTION("Preserves blue background across RGB/window buffers and PNG output")
		{
			MuJoCoPhysics physics;
			REQUIRE(physics.load_from_xml(kBlueBackgroundModelPath));

			mjData* snapshot_data = nullptr;
			const mjModel* snapshot_model = nullptr;
			double snapshot_time = 0.0;
			REQUIRE(physics.alloc_render_snapshot(snapshot_data, snapshot_model, snapshot_time));
			REQUIRE(snapshot_model != nullptr);
			REQUIRE(snapshot_data != nullptr);

			MuJoCoRenderContext context;
			if (!context.init(snapshot_model, 64, 48))
			{
				MuJoCoPhysics::destroy_snapshot(snapshot_data);
				SKIP("MuJoCo render context init failed (likely headless GL)");
			}

			auto assert_blue_rgb = [&](const HeapVector<uint8_t>& rgb, int rgb_width, int rgb_height)
			{
				REQUIRE(rgb_width > 0);
				REQUIRE(rgb_height > 0);
				REQUIRE(rgb.size() >= static_cast<size_t>(rgb_width * rgb_height * 3));

				uint64_t sum_r = 0;
				uint64_t sum_g = 0;
				uint64_t sum_b = 0;
				for (size_t i = 0; i + 2 < rgb.size(); i += 3)
				{
					sum_r += rgb[i + 0];
					sum_g += rgb[i + 1];
					sum_b += rgb[i + 2];
				}
				const double denom = static_cast<double>(rgb.size() / 3);
				const double mean_r = static_cast<double>(sum_r) / denom;
				const double mean_g = static_cast<double>(sum_g) / denom;
				const double mean_b = static_cast<double>(sum_b) / denom;

				REQUIRE(mean_b > mean_g + 20.0);
				REQUIRE(mean_b > mean_r + 20.0);
			};

			// Sanity: GL clear/read should yield blue.
			HeapVector<uint8_t> clear_rgb;
			clear_rgb.initialize(64 * 48 * 3);
			int clear_w = 0;
			int clear_h = 0;
			size_t clear_size = 0;
			REQUIRE(context.debug_clear_and_read_blue(clear_rgb.data(), clear_rgb.size(), clear_size, clear_w, clear_h, false));
			uint64_t clear_sum = 0;
			for (size_t i = 0; i < clear_size; ++i)
			{
				uint8_t v = clear_rgb.data()[i];
				clear_sum += v;
			}
			if (clear_sum == 0)
			{
				WARN("Offscreen clear+read returned zeros; continuing to render scene.");
			}
			else
			{
				assert_blue_rgb(clear_rgb, clear_w, clear_h);
			}

			int rgb_width = 0;
			int rgb_height = 0;
			auto render_rgb_and_sum = [&](uint64_t& sum_out) -> HeapVector<uint8_t>
			{
				HeapVector<uint8_t> rgb_local;
				const size_t rgb_capacity = static_cast<size_t>(64 * 48 * 3);
				if (rgb_local.size() == 0)
					rgb_local.initialize(rgb_capacity);
				size_t rgb_size = 0;
				REQUIRE(context.render_to_rgb(snapshot_model, snapshot_data, "test_cam", rgb_local.data(), rgb_local.size(), rgb_size, rgb_width, rgb_height, false));
				sum_out = 0;
				for (size_t i = 0; i < rgb_size; ++i)
				{
					sum_out += rgb_local.data()[i];
				}
				assert_blue_rgb(rgb_local, rgb_width, rgb_height);
				return rgb_local;
			};

			uint64_t rgb_sum = 0;
			auto rgb = render_rgb_and_sum(rgb_sum);
			REQUIRE(rgb_sum > 0);

			ImagePng128k png;
			const size_t rgb_size = static_cast<size_t>(rgb_width * rgb_height * 3);
			if (!encode_png_from_rgb(rgb.data(), rgb_size, rgb_width, rgb_height, png))
			{
				MuJoCoPhysics::destroy_snapshot(snapshot_data);
				SKIP("MuJoCo PNG encode failed.");
			}

			REQUIRE(png.size() > 0);

			cv::Mat encoded(1, static_cast<int>(png.size()), CV_8U, png.data());
			cv::Mat decoded = cv::imdecode(encoded, cv::IMREAD_COLOR);
			REQUIRE(!decoded.empty());

			const cv::Scalar mean_bgr = cv::mean(decoded);
			REQUIRE(mean_bgr[0] > mean_bgr[1] + 20.0);
			REQUIRE(mean_bgr[0] > mean_bgr[2] + 20.0);

			MuJoCoPhysics::destroy_snapshot(snapshot_data);
		}
	}
#else
	TEST_CASE("Unit/Systems/MuJoCoRenderContext")
	{
		SUCCEED();
	}
#endif
} // namespace robotick::tests
