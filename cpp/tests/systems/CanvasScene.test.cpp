// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#include "robotick/systems/Canvas.h"

#include <catch2/catch_all.hpp>

namespace robotick::test
{
	namespace
	{
		constexpr char kCanvasPath[] = ROBOTICK_CORE_ROOT "/cpp/tests/data/canvas/simple.canvas.yaml";

		FieldDescriptor* find_field(HeapVector<FieldDescriptor>& fields, const char* name)
		{
			for (FieldDescriptor& field : fields)
			{
				if (field.name == name)
					return &field;
			}
			return nullptr;
		}
	} // namespace

	TEST_CASE("Unit/Systems/CanvasScene/LoadAndControls")
	{
		CanvasScene scene;
		REQUIRE(scene.load_from_file(kCanvasPath));

		HeapVector<FieldDescriptor> fields;
		scene.build_control_field_descriptors(fields);

		Blackboard controls;
		controls.initialize_fields(fields);
		scene.bind_control_fields(fields);
		scene.set_control_defaults(controls);

		SECTION("Surface configuration matches YAML")
		{
			const CanvasSurface& surface = scene.surface();
			CHECK(surface.logical_width == Catch::Approx(320.0f));
			CHECK(surface.logical_height == Catch::Approx(240.0f));
			CHECK(surface.output_width == Catch::Approx(800.0f));
			CHECK(surface.output_height == Catch::Approx(480.0f));
			CHECK(surface.background.r == 255);
		}

		SECTION("Default control values populate nodes")
		{
			scene.apply_control_values(controls);

			const CanvasNode* left_eye = scene.find_node("left_eye");
			REQUIRE(left_eye != nullptr);
			CHECK(left_eye->translate.x == Catch::Approx(60.0f));
			CHECK(left_eye->translate.y == Catch::Approx(120.0f));
		}

		SECTION("Control updates propagate into canvas scene")
		{
			scene.apply_control_values(controls);

			FieldDescriptor* left_eye_translate = find_field(fields, "left_eye_translate");
			REQUIRE(left_eye_translate != nullptr);
			controls.set<Vec2f>(*left_eye_translate, Vec2f(100.0f, 75.0f));

			scene.apply_control_values(controls);
			const CanvasNode* left_eye = scene.find_node("left_eye");
			REQUIRE(left_eye != nullptr);
			CHECK(left_eye->translate.x == Catch::Approx(100.0f));
			CHECK(left_eye->translate.y == Catch::Approx(75.0f));
		}
	}

} // namespace robotick::test
