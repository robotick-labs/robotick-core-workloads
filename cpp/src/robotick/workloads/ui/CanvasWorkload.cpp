// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#include "robotick/api.h"
#include "robotick/framework/containers/HeapVector.h"
#include "robotick/framework/data/Blackboard.h"
#include "robotick/framework/strings/FixedString.h"
#include "robotick/systems/Canvas.h"
#include "robotick/systems/Image.h"
#include "robotick/systems/Renderer.h"

namespace robotick
{
	struct CanvasConfig
	{
		FixedString256 scene_path;
		bool render_to_texture = false;
	};

	struct CanvasInputs
	{
		Blackboard controls;
	};

	struct CanvasOutputs
	{
		ImagePng128k face_png_data;
	};

	struct CanvasState
	{
		bool renderer_initialized = false;
		bool scene_loaded = false;
		Renderer renderer;
		CanvasScene scene;
		FixedString256 loaded_scene_path;
		HeapVector<FieldDescriptor> control_fields;
	};

	namespace
	{
		template <typename T> void reset_heap_vector(HeapVector<T>& vec)
		{
			vec.~HeapVector<T>();
			new (&vec) HeapVector<T>();
		}
	} // namespace

	struct CanvasWorkload
	{
		CanvasConfig config;
		CanvasInputs inputs;
		CanvasOutputs outputs;
		State<CanvasState> state;

		void load_scene_from_file(const char* path)
		{
			CanvasState& s = state.get();
			if (!s.scene.load_from_file(path))
			{
				ROBOTICK_FATAL_EXIT("CanvasWorkload failed to load scene file: %s", path);
			}

			reset_heap_vector(s.control_fields);
			s.scene.build_control_field_descriptors(s.control_fields);
			inputs.controls.initialize_fields(s.control_fields);
		}

		void pre_load()
		{
			CanvasState& s = state.get();
			const char* path = config.scene_path.c_str();
			if (!path || path[0] == '\0')
			{
				ROBOTICK_FATAL_EXIT("CanvasWorkload requires config.scene_path.");
			}

			if (s.scene_loaded)
			{
				if (s.loaded_scene_path != config.scene_path)
				{
					ROBOTICK_FATAL_EXIT("CanvasWorkload scene_path changed after initialization.");
				}
				return;
			}

			load_scene_from_file(path);
			s.scene_loaded = true;
			s.loaded_scene_path = config.scene_path;
		}

		void load()
		{
			CanvasState& s = state.get();
			s.scene.bind_control_fields(s.control_fields);
			s.scene.set_control_defaults(inputs.controls);
		}

		void start(float)
		{
			CanvasState& s = state.get();

			ROBOTICK_ASSERT_MSG(s.scene_loaded, "CanvasWorkload start() called without successfully loading scene");

			if (!s.renderer_initialized)
			{
				const CanvasSurface& surface = s.scene.surface();
				s.renderer.set_texture_only_size(surface.output_width, surface.output_height);
				s.renderer.set_viewport(surface.logical_width, surface.logical_height);
				s.renderer.init(config.render_to_texture);
				s.renderer_initialized = true;
			}
		}

		void tick(const TickInfo&)
		{
			CanvasState& s = state.get();

			if (!s.scene.root())
				return;

			s.scene.apply_control_values(inputs.controls);
			s.renderer.clear(s.scene.surface().background);
			s.scene.draw(s.renderer);

			if (config.render_to_texture)
			{
				size_t png_size = 0;
				if (s.renderer.capture_as_png(outputs.face_png_data.data(), outputs.face_png_data.capacity(), png_size))
				{
					outputs.face_png_data.set_size(png_size);
				}
				else
				{
					outputs.face_png_data.set_size(0);
				}
			}
			else
			{
				s.renderer.present();
				outputs.face_png_data.set_size(0);
			}
		}

		void stop() {}
	};

} // namespace robotick
