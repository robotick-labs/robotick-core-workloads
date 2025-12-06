// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#if defined(ROBOTICK_PLATFORM_DESKTOP)

#include "robotick/systems/MqttFieldSync.h"
#include "robotick/api.h"
#include "robotick/framework/memory/Memory.h"
#include "robotick/framework/memory/StdApproved.h"

namespace robotick
{
	static const char* mqtt_op_result_str(MqttOpResult result)
	{
		switch (result)
		{
		case MqttOpResult::Success:
			return "success";
		case MqttOpResult::Dropped:
			return "dropped";
		case MqttOpResult::Error:
		default:
			return "error";
		}
	}
	namespace
	{
		bool starts_with(const char* text, const char* prefix)
		{
			if (!text || !prefix)
				return false;

			while (*prefix)
			{
				if (*text == '\0' || *text != *prefix)
					return false;
				++text;
				++prefix;
			}
			return true;
		}

		void replace_characters(FixedString512& str, char from, char to)
		{
			char* data = str.str();
			for (size_t i = 0; data[i] != '\0'; ++i)
			{
				if (data[i] == from)
					data[i] = to;
			}
		}
	} // namespace

	MqttFieldSync::MqttFieldSync(const char* root_ns, PublisherFn in_publisher)
		: publisher(robotick::move(in_publisher))
		, mqtt_ptr(nullptr)
		, engine_ptr(nullptr)
	{
		root.assign(root_ns, root_ns ? fixed_strlen(root_ns) : 0);
	}

	MqttFieldSync::MqttFieldSync(Engine& engine, const char* root_ns, IMqttClient& mqtt_client)
		: publisher(nullptr)
		, mqtt_ptr(&mqtt_client)
		, engine_ptr(&engine)
	{
		root.assign(root_ns, root_ns ? fixed_strlen(root_ns) : 0);

		try
		{
			mqtt_ptr->set_callback(
				[this](const char* topic, const char* payload)
				{
					FixedString512 control_prefix;
					control_prefix.format("%s/control/", root.c_str());

					if (!topic_starts_with(topic, control_prefix.c_str()))
						return;

					nlohmann::json incoming;
					try
					{
						incoming = nlohmann::json::parse(payload);
					}
					catch (...)
					{
						ROBOTICK_WARNING("MqttFieldSync - Ignoring malformed JSON from topic: %s", topic ? topic : "(null)");
						return;
					}

					FixedString256 topic_key;
					topic_key.assign(topic, topic ? fixed_strlen(topic) : 0);

					auto* previous = last_published.find(topic_key);
					if (previous && *previous == incoming)
						return;

					queue_control_topic(topic, incoming);
				});
		}
		catch (...)
		{
			ROBOTICK_FATAL_EXIT("MqttFieldSync - Failed to set MQTT callback.");
		}
	}

	bool MqttFieldSync::topic_starts_with(const char* topic, const char* prefix) const
	{
		return starts_with(topic, prefix);
	}

	void MqttFieldSync::queue_control_topic(const char* topic, const nlohmann::json& value)
	{
		store_topic(updated_topics, topic, value);
	}

	void MqttFieldSync::store_topic(TopicMap& table, const char* topic, const nlohmann::json& value)
	{
		FixedString256 key;
		key.assign(topic, topic ? fixed_strlen(topic) : 0);

		if (auto* existing = table.find(key))
		{
			*existing = value;
		}
		else
		{
			table.insert(key, value);
		}
	}

	MqttOpResult MqttFieldSync::subscribe_and_sync_startup()
	{
		ROBOTICK_ASSERT_MSG(mqtt_ptr != nullptr, "MqttFieldSync::subscribe_and_sync_startup - mqtt_ptr should have been set before calling");
		ROBOTICK_ASSERT_MSG(engine_ptr != nullptr, "MqttFieldSync::subscribe_and_sync_startup - engine_ptr should have been set before calling");

		FixedString512 control_topic;
		control_topic.format("%s/control/#", root.c_str());

		const MqttOpResult sub_result = mqtt_ptr->subscribe(control_topic.c_str());
		metrics.last_subscribe_result = sub_result;
		metrics.last_control_result = sub_result;
		if (sub_result != MqttOpResult::Success)
		{
			ROBOTICK_WARNING("MqttFieldSync - Failed to subscribe to control topics (%s).", mqtt_op_result_str(sub_result));
			metrics.subscribe_failures++;
		}

		publish_fields(*engine_ptr, engine_ptr->get_workloads_buffer(), true);

		updated_topics.clear();
		return sub_result;
	}

	void MqttFieldSync::apply_control_updates()
	{
		if (!engine_ptr)
			return;

		FixedString512 control_prefix;
		control_prefix.format("%s/control/", root.c_str());
		const size_t prefix_len = control_prefix.length();

		updated_topics.for_each(
			[&](const FixedString256& topic_key, nlohmann::json& json_value)
			{
				const char* topic_cstr = topic_key.c_str();
				if (!topic_starts_with(topic_cstr, control_prefix.c_str()))
					return;

				FixedString512 path;
				path.assign(topic_cstr + prefix_len, topic_key.length() - prefix_len);
				replace_characters(path, '/', '.');

				FieldInfo info = DataConnectionUtils::find_field_info(*engine_ptr, path.c_str());
				if (!info.ptr || !info.descriptor)
				{
					ROBOTICK_WARNING("MqttFieldSync::apply_control_updates() - unable to resolve field path: %s", path.c_str());
					return;
				}

				const TypeDescriptor* type_desc = info.descriptor->find_type_descriptor();
				if (!type_desc)
				{
					ROBOTICK_WARNING("MqttFieldSync::apply_control_updates() - field '%s' has no registered type descriptor", path.c_str());
					return;
				}

				const std_approved::string value_str = json_value.dump();
				if (!type_desc->from_string(value_str.c_str(), info.ptr))
				{
					ROBOTICK_WARNING(
						"MqttFieldSync::apply_control_updates() - failed to parse value '%s' for field '%s'", value_str.c_str(), path.c_str());
					return;
				}
			});

		updated_topics.clear();
	}

	void MqttFieldSync::publish_state_fields()
	{
		if (!engine_ptr)
			return;
		publish_fields(*engine_ptr, engine_ptr->get_workloads_buffer(), false);
	}

	void MqttFieldSync::publish_fields(const Engine& engine, const WorkloadsBuffer& buffer, bool publish_control)
	{
		WorkloadsBuffer& non_const_buf = const_cast<WorkloadsBuffer&>(buffer);

		WorkloadFieldsIterator::for_each_workload_field(engine,
			&non_const_buf,
			[&](const WorkloadFieldView& top_view)
			{
				if (!top_view.workload_info || !top_view.struct_info || !top_view.field_info)
					return;

				const WorkloadDescriptor* workload_desc = top_view.workload_info->type->get_workload_desc();
				if (!workload_desc)
					return;

				bool is_struct_read_only = true;
				const char* struct_name_cstr = nullptr;

				if (top_view.struct_info == workload_desc->config_desc)
				{
					struct_name_cstr = "config";
				}
				else if (top_view.struct_info == workload_desc->inputs_desc)
				{
					struct_name_cstr = "inputs";
					is_struct_read_only = false;
				}
				else if (top_view.struct_info == workload_desc->outputs_desc)
				{
					struct_name_cstr = "outputs";
				}
				else
				{
					return;
				}

				const char* workload_name = top_view.workload_info->seed->unique_name;
				FixedString512 base_path;
				base_path.format("%s/%s/%s", workload_name, struct_name_cstr, top_view.field_info->name.c_str());

				auto visit_leafs = [&](auto&& self, const WorkloadFieldView& view, const FixedString512& path_so_far) -> void
				{
					if (view.is_struct_field())
					{
						WorkloadFieldsIterator::for_each_field_in_struct_field(view,
							[&](const WorkloadFieldView& child)
							{
								const char* child_name = child.subfield_info ? child.subfield_info->name.c_str()
																			 : (child.field_info ? child.field_info->name.c_str() : "(unknown)");
								FixedString512 next_path(path_so_far.c_str());
								next_path.append("/");
								next_path.append(child_name);
								self(self, child, next_path);
							});
						return;
					}

					if (!view.field_ptr)
						return;

					TypeId type = view.subfield_info ? view.subfield_info->type_id : view.field_info ? view.field_info->type_id : TypeId{};
					if (!type.is_valid())
						return;

					nlohmann::json value = serialize(view.field_ptr, type);

					FixedString512 state_topic;
					state_topic.format("%s/state/%s", root.c_str(), path_so_far.c_str());
					store_topic(last_published, state_topic.c_str(), value);

					FixedString1024 payload;
					const std_approved::string dumped = value.dump();
					payload.assign(dumped.c_str(), dumped.size());

					if (mqtt_ptr)
					{
						const MqttOpResult pub_res = mqtt_ptr->publish(state_topic.c_str(), payload.c_str(), true);
						metrics.last_state_result = pub_res;
						if (pub_res != MqttOpResult::Success)
						{
							metrics.state_publish_failures++;
							ROBOTICK_WARNING(
								"MqttFieldSync - Failed to publish state topic %s (%s)", state_topic.c_str(), mqtt_op_result_str(pub_res));
						}
					}
					else if (publisher)
					{
						FixedString512 relative_topic;
						relative_topic.format("state/%s", path_so_far.c_str());
						publisher(relative_topic.c_str(), payload.c_str(), true);
					}

					if (publish_control && !is_struct_read_only)
					{
						FixedString512 control_topic;
						control_topic.format("%s/control/%s", root.c_str(), path_so_far.c_str());
						store_topic(last_published, control_topic.c_str(), value);

						if (mqtt_ptr)
						{
							const MqttOpResult control_res = mqtt_ptr->publish(control_topic.c_str(), payload.c_str(), true);
							metrics.last_control_result = control_res;
							if (control_res != MqttOpResult::Success)
							{
								metrics.control_publish_failures++;
								ROBOTICK_WARNING("MqttFieldSync - Failed to publish control topic %s (%s)",
									control_topic.c_str(),
									mqtt_op_result_str(control_res));
							}
						}
						else if (publisher)
						{
							FixedString512 relative_topic;
							relative_topic.format("control/%s", path_so_far.c_str());
							publisher(relative_topic.c_str(), payload.c_str(), true);
						}
					}
				};

				visit_leafs(visit_leafs, top_view, base_path);
			});
	}

	nlohmann::json MqttFieldSync::serialize(void* ptr, TypeId type)
	{
		if (type == GET_TYPE_ID(int))
			return *reinterpret_cast<int*>(ptr);
		if (type == GET_TYPE_ID(double))
			return *reinterpret_cast<double*>(ptr);
		if (type == GET_TYPE_ID(FixedString64))
			return reinterpret_cast<FixedString64*>(ptr)->c_str();
		if (type == GET_TYPE_ID(FixedString128))
			return reinterpret_cast<FixedString128*>(ptr)->c_str();
		return nullptr;
	}

} // namespace robotick

#else

#include "robotick/systems/MqttFieldSync.h"

namespace robotick
{
	MqttFieldSync::MqttFieldSync(const char*, PublisherFn in_publisher)
		: publisher(robotick::move(in_publisher))
		, mqtt_ptr(nullptr)
		, engine_ptr(nullptr)
	{
	}

	MqttFieldSync::MqttFieldSync(Engine&, const char*, IMqttClient&)
		: publisher(nullptr)
		, mqtt_ptr(nullptr)
		, engine_ptr(nullptr)
	{
	}

	MqttOpResult MqttFieldSync::subscribe_and_sync_startup()
	{
		return MqttOpResult::Success;
	}

	void MqttFieldSync::apply_control_updates()
	{
	}

	void MqttFieldSync::publish_state_fields()
	{
	}

	void MqttFieldSync::publish_fields(const Engine&, const WorkloadsBuffer&, bool)
	{
	}

	void MqttFieldSync::queue_control_topic(const char*, const nlohmann::json&)
	{
	}

	nlohmann::json MqttFieldSync::serialize(void*, TypeId)
	{
		return nlohmann::json();
	}

	void MqttFieldSync::store_topic(TopicMap&, const char*, const nlohmann::json&)
	{
	}

	bool MqttFieldSync::topic_starts_with(const char*, const char*) const
	{
		return false;
	}
} // namespace robotick

#endif // ROBOTICK_PLATFORM_DESKTOP
