// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#include "robotick/systems/Canvas.h"

#include "robotick/framework/math/Abs.h"
#include "robotick/framework/math/MathUtils.h"
#include "robotick/framework/memory/Memory.h"

#if defined(ROBOTICK_PLATFORM_LINUX)
#include <yaml-cpp/yaml.h>
#endif

#include <cmath>
#include <cstring>
#include <new>

namespace robotick
{
	namespace
	{
		void reset_node(CanvasNode& node)
		{
			node = CanvasNode{};
		}

		Vec2f rotate_vec(const Vec2f& v, float degrees)
		{
			if (robotick::abs(degrees) < 1e-4f)
				return v;
			const float radians = deg_to_rad(degrees);
			const float c = cosf(radians);
			const float s = sinf(radians);
			return Vec2f(v.x * c - v.y * s, v.x * s + v.y * c);
		}

#if defined(ROBOTICK_PLATFORM_LINUX)
		size_t count_nodes_recursive(const YAML::Node& node)
		{
			if (!node || !node.IsMap())
				return 0;

			size_t total = 1;
			if (const YAML::Node children = node["children"])
			{
				if (!children.IsSequence())
					ROBOTICK_FATAL_EXIT("Canvas node 'children' must be a sequence.");
				for (const YAML::Node& child : children)
				{
					total += count_nodes_recursive(child);
				}
			}
			return total;
		}

		size_t count_controls(const YAML::Node& controls_node)
		{
			if (!controls_node)
				return 0;
			if (!controls_node.IsSequence())
				ROBOTICK_FATAL_EXIT("CanvasWorkload 'controls' must be a sequence.");
			return controls_node.size();
		}

		Vec2f parse_vec2(const YAML::Node& node, const Vec2f& default_value)
		{
			if (!node)
				return default_value;

			if (node.IsScalar())
			{
				const float v = node.as<float>();
				return Vec2f(v, v);
			}

			Vec2f result = default_value;
			if (node["x"])
				result.x = node["x"].as<float>();
			if (node["y"])
				result.y = node["y"].as<float>();
			return result;
		}

		Color parse_color(const YAML::Node& node, const Color& default_color)
		{
			if (!node || !node.IsMap())
			{
				return default_color;
			}

			Color color = default_color;
			if (node["r"])
				color.r = static_cast<uint8_t>(robotick::clamp(node["r"].as<int>(), 0, 255));
			if (node["g"])
				color.g = static_cast<uint8_t>(robotick::clamp(node["g"].as<int>(), 0, 255));
			if (node["b"])
				color.b = static_cast<uint8_t>(robotick::clamp(node["b"].as<int>(), 0, 255));
			if (node["a"])
				color.a = static_cast<uint8_t>(robotick::clamp(node["a"].as<int>(), 0, 255));
			return color;
		}

		CanvasNodeType parse_node_type(const YAML::Node& node)
		{
			if (!node || !node.IsScalar())
				return CanvasNodeType::Group;

			const auto value = node.Scalar();
			if (value == "group" || value == "locator")
				return CanvasNodeType::Group;
			if (value == "ellipse")
				return CanvasNodeType::Ellipse;
			if (value == "rect")
				return CanvasNodeType::Rect;

			ROBOTICK_FATAL_EXIT("Unknown canvas node type '%s'. Supported: group, ellipse, rect.", value.c_str());
			return CanvasNodeType::Group;
		}
#endif

		float clamp01(float value)
		{
			return robotick::clamp(value, 0.0f, 1.0f);
		}
	} // namespace

	CanvasScene::CanvasScene() = default;
	CanvasScene::~CanvasScene() = default;

	bool CanvasScene::load_from_file(const char* path)
	{
#if !defined(ROBOTICK_PLATFORM_LINUX)
		(void)path;
		ROBOTICK_WARNING("CanvasScene::load_from_file is not supported on this platform (yaml-cpp unavailable).");
		return false;
#else
		if (root_ != nullptr || nodes_.size() > 0)
		{
			ROBOTICK_FATAL_EXIT("CanvasScene already loaded. Create a new CanvasScene for each scene.");
		}

		YAML::Node root_yaml;
		try
		{
			root_yaml = YAML::LoadFile(path);
		}
		catch (const YAML::BadFile&)
		{
			return false;
		}

		if (!root_yaml)
			return false;

		const YAML::Node canvas_node = root_yaml["canvas"];
		if (!canvas_node || !canvas_node.IsMap())
		{
			ROBOTICK_FATAL_EXIT("Canvas scene missing required 'canvas' map.");
		}

		const YAML::Node scene_node = root_yaml["scene"];
		if (!scene_node || !scene_node.IsMap())
		{
			ROBOTICK_FATAL_EXIT("Canvas scene missing required 'scene' root node.");
		}

		parse_canvas_config(canvas_node);

		const size_t node_count = count_nodes_recursive(scene_node);
		if (node_count == 0)
		{
			ROBOTICK_FATAL_EXIT("Canvas scene must contain at least one node.");
		}

		nodes_.initialize(node_count);

		node_lookup_.initialize(node_count);

		const size_t control_count = count_controls(root_yaml["controls"]);
		if (control_count > 0)
		{
			control_bindings_.initialize(control_count);
			control_aliases_.initialize(control_count);
		}

		size_t next_node_index = 0;
		root_ = parse_node_recursive(scene_node, next_node_index);

		size_t next_lookup_index = 0;
		populate_lookup(*root_, next_lookup_index);

		parse_controls(root_yaml["controls"]);

		source_path_ = path;
		return true;
#endif
	}

	const CanvasNode* CanvasScene::find_node(StringView id) const
	{
		for (const NodeLookupEntry& entry : node_lookup_)
		{
			if (entry.id == id.c_str())
				return entry.node;
		}
		return nullptr;
	}

	void CanvasScene::build_control_field_descriptors(HeapVector<FieldDescriptor>& out_fields) const
	{
		const size_t control_count = control_bindings_.size();
		out_fields.initialize(control_count);
		size_t offset = 0;

		for (size_t i = 0; i < control_count; ++i)
		{
			FieldDescriptor& field = out_fields[i];
			field.name = control_aliases_.size() > i ? control_aliases_[i] : nullptr;

			switch (control_bindings_[i].property)
			{
			case ControlProperty::Translate:
			case ControlProperty::Scale:
				field.type_id = GET_TYPE_ID(Vec2f);
				break;
			case ControlProperty::TranslateX:
			case ControlProperty::TranslateY:
			case ControlProperty::ScaleX:
			case ControlProperty::ScaleY:
			case ControlProperty::RotateDeg:
			case ControlProperty::Alpha:
				field.type_id = GET_TYPE_ID(float);
				break;
			case ControlProperty::Visible:
				field.type_id = GET_TYPE_ID(bool);
				break;
			}

			const TypeDescriptor* type_desc = field.find_type_descriptor();
			if (!type_desc)
			{
				ROBOTICK_FATAL_EXIT("Unable to resolve type descriptor for Canvas control '%s'.", field.name.c_str());
			}

			const size_t alignment = type_desc->alignment;
			offset = (offset + alignment - 1) & ~(alignment - 1);
			field.offset_within_container = offset;
			offset += type_desc->size;
		}
	}

	void CanvasScene::bind_control_fields(HeapVector<FieldDescriptor>& fields)
	{
		const size_t count = control_bindings_.size();
		for (size_t i = 0; i < count && i < fields.size(); ++i)
		{
			control_bindings_[i].field = &fields[i];
		}
	}

	void CanvasScene::set_control_defaults(Blackboard& controls) const
	{
		for (const ControlBinding& binding : control_bindings_)
		{
			if (!binding.field)
				continue;

			switch (binding.property)
			{
			case ControlProperty::Translate:
				controls.set<Vec2f>(*binding.field, binding.node->translate);
				break;
			case ControlProperty::TranslateX:
				controls.set<float>(*binding.field, binding.node->translate.x);
				break;
			case ControlProperty::TranslateY:
				controls.set<float>(*binding.field, binding.node->translate.y);
				break;
			case ControlProperty::Scale:
				controls.set<Vec2f>(*binding.field, binding.node->scale);
				break;
			case ControlProperty::ScaleX:
				controls.set<float>(*binding.field, binding.node->scale.x);
				break;
			case ControlProperty::ScaleY:
				controls.set<float>(*binding.field, binding.node->scale.y);
				break;
			case ControlProperty::RotateDeg:
				controls.set<float>(*binding.field, binding.node->rotate_deg);
				break;
			case ControlProperty::Visible:
				controls.set<bool>(*binding.field, binding.node->visible);
				break;
			case ControlProperty::Alpha:
				controls.set<float>(*binding.field, binding.node->alpha);
				break;
			}
		}
	}

	void CanvasScene::apply_control_values(const Blackboard& controls)
	{
		for (ControlBinding& binding : control_bindings_)
		{
			if (!binding.field)
				continue;

			switch (binding.property)
			{
			case ControlProperty::Translate:
				binding.node->translate = controls.get<Vec2f>(*binding.field);
				break;
			case ControlProperty::TranslateX:
				binding.node->translate.x = controls.get<float>(*binding.field);
				break;
			case ControlProperty::TranslateY:
				binding.node->translate.y = controls.get<float>(*binding.field);
				break;
			case ControlProperty::Scale:
				binding.node->scale = controls.get<Vec2f>(*binding.field);
				break;
			case ControlProperty::ScaleX:
				binding.node->scale.x = controls.get<float>(*binding.field);
				break;
			case ControlProperty::ScaleY:
				binding.node->scale.y = controls.get<float>(*binding.field);
				break;
			case ControlProperty::RotateDeg:
				binding.node->rotate_deg = controls.get<float>(*binding.field);
				break;
			case ControlProperty::Visible:
				binding.node->visible = controls.get<bool>(*binding.field);
				break;
			case ControlProperty::Alpha:
				binding.node->alpha = controls.get<float>(*binding.field);
				break;
			}
		}
	}

	void CanvasScene::draw(Renderer& renderer) const
	{
		if (!root_)
			return;

		draw_node_recursive(*root_, Vec2f(0.0f, 0.0f), Vec2f(1.0f, 1.0f), 0.0f, true, 1.0f, renderer);
	}

#if defined(ROBOTICK_PLATFORM_LINUX)
	void CanvasScene::parse_canvas_config(const YAML::Node& canvas_node)
	{
		if (const YAML::Node logical = canvas_node["logical_size"])
		{
			surface_.logical_width = logical["width"].as<float>(surface_.logical_width);
			surface_.logical_height = logical["height"].as<float>(surface_.logical_height);
		}

		if (const YAML::Node output = canvas_node["output_size"])
		{
			surface_.output_width = output["width"].as<float>(surface_.output_width);
			surface_.output_height = output["height"].as<float>(surface_.output_height);
		}

		surface_.background = parse_color(canvas_node["background"], surface_.background);
	}

	CanvasNode* CanvasScene::parse_node_recursive(const YAML::Node& yaml_node, size_t& next_index)
	{
		if (!yaml_node || !yaml_node.IsMap())
			ROBOTICK_FATAL_EXIT("Each node entry must be a map.");

		if (next_index >= nodes_.size())
		{
			ROBOTICK_FATAL_EXIT("Canvas node allocation exhausted.");
		}

		CanvasNode* node = &nodes_[next_index++];
		reset_node(*node);

		const YAML::Node id_node = yaml_node["id"];
		if (!id_node || !id_node.IsScalar())
		{
			ROBOTICK_FATAL_EXIT("Node is missing required 'id' scalar.");
		}
		const auto id_scalar = id_node.Scalar();
		node->id.assign(id_scalar.c_str(), id_scalar.size());
		node->type = parse_node_type(yaml_node["type"]);
		node->translate = parse_vec2(yaml_node["translate"], Vec2f(0.0f, 0.0f));
		node->rotate_deg = yaml_node["rotate_deg"].as<float>(0.0f);
		node->scale = parse_vec2(yaml_node["scale"], Vec2f(1.0f, 1.0f));
		node->visible = yaml_node["visible"].as<bool>(true);
		node->alpha = yaml_node["alpha"].as<float>(1.0f);

		if (const YAML::Node style_node = yaml_node["style"])
		{
			if (const YAML::Node fill_node = style_node["fill"])
			{
				node->fill = parse_color(fill_node, node->fill);
				node->has_fill = true;
			}
		}

		if (node->type == CanvasNodeType::Ellipse)
		{
			const YAML::Node geo = yaml_node["geometry"];
			if (!geo || !geo.IsMap())
				ROBOTICK_FATAL_EXIT("Ellipse node '%s' requires geometry map.", node->id.c_str());

			if (!geo["rx"] || !geo["ry"])
			{
				ROBOTICK_FATAL_EXIT("Ellipse node '%s' geometry must contain rx/ry.", node->id.c_str());
			}
			node->ellipse_rx = geo["rx"].as<float>();
			node->ellipse_ry = geo["ry"].as<float>();
		}
		else if (node->type == CanvasNodeType::Rect)
		{
			const YAML::Node geo = yaml_node["geometry"];
			if (!geo || !geo.IsMap())
				ROBOTICK_FATAL_EXIT("Rect node '%s' requires geometry map.", node->id.c_str());
			if (!geo["w"] || !geo["h"])
			{
				ROBOTICK_FATAL_EXIT("Rect node '%s' geometry must contain w/h.", node->id.c_str());
			}
			node->rect_w = geo["w"].as<float>();
			node->rect_h = geo["h"].as<float>();
		}

		if (const YAML::Node children = yaml_node["children"])
		{
			if (!children.IsSequence())
				ROBOTICK_FATAL_EXIT("Node '%s' children must be a sequence.", node->id.c_str());

			const size_t child_count = children.size();
			if (child_count > 0)
			{
				node->children.initialize(child_count);
				size_t child_index = 0;
				for (const YAML::Node& child_yaml : children)
				{
					node->children[child_index++] = parse_node_recursive(child_yaml, next_index);
				}
			}
		}

		return node;
	}

	void CanvasScene::populate_lookup(CanvasNode& node, size_t& next_index)
	{
		if (node.id.empty())
			ROBOTICK_FATAL_EXIT("Canvas node id cannot be empty.");

		if (next_index >= node_lookup_.size())
			ROBOTICK_FATAL_EXIT("Canvas lookup allocation exhausted.");

		for (size_t i = 0; i < next_index; ++i)
		{
			if (node_lookup_[i].id == node.id.c_str())
			{
				ROBOTICK_FATAL_EXIT("Duplicate canvas node id '%s'.", node.id.c_str());
			}
		}

		NodeLookupEntry& entry = node_lookup_[next_index++];
		entry.id = node.id.c_str();
		entry.node = &node;

		for (CanvasNode* child : node.children)
		{
			populate_lookup(*child, next_index);
		}
	}

	void CanvasScene::parse_controls(const YAML::Node& controls_node)
	{
		if (!controls_node)
			return;

		if (!controls_node.IsSequence())
		{
			ROBOTICK_FATAL_EXIT("CanvasWorkload 'controls' must be a sequence.");
		}

		if (controls_node.size() != control_bindings_.size())
		{
			ROBOTICK_FATAL_EXIT("Canvas control count mismatch.");
		}

		size_t index = 0;
		for (const YAML::Node& entry : controls_node)
		{
			if (!entry || !entry.IsMap())
				ROBOTICK_FATAL_EXIT("CanvasWorkload controls entries must be maps.");

			const YAML::Node target_node = entry["target"];
			const YAML::Node alias_node = entry["alias"];
			if (!target_node || !alias_node)
				ROBOTICK_FATAL_EXIT("CanvasWorkload control entries must contain 'target' and 'alias'.");

			ControlBinding& binding = control_bindings_[index];
			parse_target(target_node.Scalar().c_str(), binding);

			const auto alias_scalar = alias_node.Scalar();
			FixedString64 alias_string(alias_scalar.c_str());
			FixedString64& stored_alias = alias_storage_.push_back(alias_string);
			if (index < control_aliases_.size())
			{
				control_aliases_[index] = stored_alias.c_str();
			}
			++index;
		}
	}

	void CanvasScene::parse_target(const char* target, ControlBinding& out_binding)
	{
		const char* dot = target ? ::strchr(target, '.') : nullptr;
		if (!dot)
		{
			ROBOTICK_FATAL_EXIT("CanvasWorkload control target '%s' missing property.", target ? target : "<null>");
		}

		FixedString64 node_id;
		node_id.assign(target, static_cast<size_t>(dot - target));
		const char* property_path = dot + 1;

		CanvasNode* node = const_cast<CanvasNode*>(find_node(node_id.c_str()));
		if (!node)
		{
			ROBOTICK_FATAL_EXIT("CanvasWorkload control target references unknown node '%s'.", node_id.c_str());
		}

		out_binding.node = node;
		out_binding.property = parse_property_path(property_path);
	}
#endif

	CanvasScene::ControlProperty CanvasScene::parse_property_path(const char* path) const
	{
		if (::strcmp(path, "translate") == 0)
			return ControlProperty::Translate;
		if (::strcmp(path, "translate.x") == 0)
			return ControlProperty::TranslateX;
		if (::strcmp(path, "translate.y") == 0)
			return ControlProperty::TranslateY;
		if (::strcmp(path, "scale") == 0)
			return ControlProperty::Scale;
		if (::strcmp(path, "scale.x") == 0)
			return ControlProperty::ScaleX;
		if (::strcmp(path, "scale.y") == 0)
			return ControlProperty::ScaleY;
		if (::strcmp(path, "rotate_deg") == 0)
			return ControlProperty::RotateDeg;
		if (::strcmp(path, "visible") == 0)
			return ControlProperty::Visible;
		if (::strcmp(path, "alpha") == 0)
			return ControlProperty::Alpha;

		ROBOTICK_FATAL_EXIT("CanvasWorkload unsupported control target property '%s'.", path);
		return ControlProperty::Translate;
	}

	void CanvasScene::draw_node_recursive(const CanvasNode& node,
		const Vec2f& parent_translate,
		const Vec2f& parent_scale,
		float parent_rotation_deg,
		bool parent_visible,
		float parent_opacity,
		Renderer& renderer) const
	{
		bool current_visible = parent_visible && node.visible;
		const float current_opacity = parent_opacity * node.alpha;

		Vec2f scaled_translate(node.translate.x * parent_scale.x, node.translate.y * parent_scale.y);
		Vec2f rotated_translate = rotate_vec(scaled_translate, parent_rotation_deg);
		Vec2f world_translate(parent_translate.x + rotated_translate.x, parent_translate.y + rotated_translate.y);

		Vec2f world_scale(parent_scale.x * node.scale.x, parent_scale.y * node.scale.y);
		const float world_rotation = parent_rotation_deg + node.rotate_deg;

		if (current_visible)
		{
			if (node.type == CanvasNodeType::Ellipse && node.has_fill)
			{
				if (robotick::abs(world_rotation) > 1e-4f)
				{
					ROBOTICK_WARNING("CanvasWorkload ellipse node '%s' rotation is not supported; ignoring rotation.", node.id.c_str());
				}

				Color color = node.fill;
				const float alpha_scale = clamp01(current_opacity);
				const float alpha = static_cast<float>(color.a) * alpha_scale;
				color.a = static_cast<uint8_t>(robotick::clamp(alpha, 0.0f, 255.0f));

				const float rx = node.ellipse_rx * robotick::abs(world_scale.x);
				const float ry = node.ellipse_ry * robotick::abs(world_scale.y);
				renderer.draw_ellipse_filled(Vec2(world_translate.x, world_translate.y), rx, ry, color);
			}
			else if (node.type == CanvasNodeType::Rect && node.has_fill)
			{
				if (robotick::abs(world_rotation) > 1e-4f)
				{
					ROBOTICK_WARNING("CanvasWorkload rect node '%s' rotation is not supported; ignoring rotation.", node.id.c_str());
				}

				Color color = node.fill;
				const float alpha_scale = clamp01(current_opacity);
				const float alpha = static_cast<float>(color.a) * alpha_scale;
				color.a = static_cast<uint8_t>(robotick::clamp(alpha, 0.0f, 255.0f));

				const float half_w = 0.5f * node.rect_w * robotick::abs(world_scale.x);
				const float half_h = 0.5f * node.rect_h * robotick::abs(world_scale.y);
				const Vec2 p0(world_translate.x - half_w, world_translate.y - half_h);
				const Vec2 p1(world_translate.x + half_w, world_translate.y + half_h);
				renderer.draw_rect_filled(p0, p1, color);
			}
		}

		for (CanvasNode* child : node.children)
		{
			draw_node_recursive(*child, world_translate, world_scale, world_rotation, current_visible, current_opacity, renderer);
		}
	}

} // namespace robotick
