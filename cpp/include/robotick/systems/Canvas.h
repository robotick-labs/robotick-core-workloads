// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "robotick/framework/containers/HeapVector.h"
#include "robotick/framework/containers/List.h"
#include "robotick/framework/data/Blackboard.h"
#include "robotick/framework/math/Vec2.h"
#include "robotick/framework/strings/FixedString.h"
#include "robotick/framework/strings/StringView.h"
#include "robotick/systems/Renderer.h"

namespace YAML
{
	class Node;
}

namespace robotick
{
	struct CanvasSurface
	{
		float logical_width = 320.0f;
		float logical_height = 240.0f;
		float output_width = 320.0f;
		float output_height = 240.0f;
		Color background = Colors::Black;
	};

	enum class CanvasNodeType
	{
		Group,
		Ellipse,
		Rect,
	};

	struct CanvasNode
	{
		FixedString64 id;
		CanvasNodeType type = CanvasNodeType::Group;
		Vec2f translate = {0.0f, 0.0f};
		float rotate_deg = 0.0f;
		Vec2f scale = {1.0f, 1.0f};
		bool visible = true;
		float alpha = 1.0f;
		bool has_fill = false;
		Color fill = Colors::Black;
		float ellipse_rx = 0.0f;
		float ellipse_ry = 0.0f;
		float rect_w = 0.0f;
		float rect_h = 0.0f;
		HeapVector<CanvasNode*> children;
	};

	class CanvasScene
	{
	  public:
		CanvasScene();
		~CanvasScene();

		bool load_from_file(const char* path);

		const CanvasSurface& surface() const { return surface_; }
		const CanvasNode* root() const { return root_; }
		const CanvasNode* find_node(StringView id) const;

		void build_control_field_descriptors(HeapVector<FieldDescriptor>& out_fields) const;
		void bind_control_fields(HeapVector<FieldDescriptor>& fields);
		void set_control_defaults(Blackboard& controls) const;
		void apply_control_values(const Blackboard& controls);
		void draw(Renderer& renderer) const;

	  private:
		enum class ControlProperty
		{
			Translate,
			TranslateX,
			TranslateY,
			Scale,
			ScaleX,
			ScaleY,
			RotateDeg,
			Visible,
			Alpha,
		};

		struct NodeLookupEntry
		{
			FixedString64 id;
			CanvasNode* node = nullptr;
		};

		struct ControlBinding
		{
			CanvasNode* node = nullptr;
			ControlProperty property = ControlProperty::Translate;
			FieldDescriptor* field = nullptr;
		};

		bool parse_scene_yaml(const char* path, const YAML::Node& root);
		void parse_canvas_config(const YAML::Node& canvas_node);
		CanvasNode* parse_node_recursive(const YAML::Node& node_yaml, size_t& next_index);
		void populate_lookup(CanvasNode& node, size_t& next_index);
		void parse_controls(const YAML::Node& controls_node);
		ControlProperty parse_property_path(const char* path) const;
		void parse_target(const char* target, ControlBinding& binding);
		void draw_node_recursive(const CanvasNode& node,
			const Vec2f& parent_translate,
			const Vec2f& parent_scale,
			float parent_rotation_deg,
			bool parent_visible,
			float parent_opacity,
			Renderer& renderer) const;

		CanvasSurface surface_;
		CanvasNode* root_ = nullptr;
		HeapVector<CanvasNode> nodes_;
		HeapVector<NodeLookupEntry> node_lookup_;
		HeapVector<ControlBinding> control_bindings_;
		List<FixedString64> alias_storage_;
		HeapVector<const char*> control_aliases_;
		FixedString256 source_path_;
	};

} // namespace robotick
