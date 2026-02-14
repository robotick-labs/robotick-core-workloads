// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#if defined(ROBOTICK_PLATFORM_DESKTOP) || defined(ROBOTICK_PLATFORM_LINUX)

#include "robotick/api.h"
#include "robotick/framework/data/Blackboard.h"
#include "robotick/framework/registry/TypeRegistry.h"
#include "robotick/framework/strings/FixedString.h"
#include "robotick/systems/PythonRuntime.h"

#include <algorithm>
#include <cassert>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <mutex>
#include <pybind11/buffer_info.h>
#include <pybind11/embed.h>
#include <pybind11/stl.h>

namespace py = pybind11;

namespace robotick
{
	namespace
	{
		template <typename FixedStringType> FixedStringType py_to_fixed_string(const py::handle& source)
		{
			FixedStringType result;
			const py::bytes encoded = py::bytes(py::str(source).attr("encode")("utf-8"));
			py::buffer_info info = py::buffer(encoded).request();

			const size_t max_copy = result.capacity() - 1;
			const size_t copy_len = robotick::min_val(static_cast<size_t>(info.size), max_copy);
			if (copy_len > 0)
			{
				memcpy(result.data, info.ptr, copy_len);
			}
			result.data[copy_len] = '\0';
			return result;
		}

		template <typename FixedStringType> void to_lower(FixedStringType& target)
		{
			const size_t len = target.length();
			for (size_t i = 0; i < len; ++i)
			{
				target.data[i] = static_cast<char>(::tolower(static_cast<unsigned char>(target.data[i])));
			}
		}

		static bool blackboard_field_to_py_dict(const Blackboard& blackboard, const FieldDescriptor& field, py::dict& py_dict)
		{
			const char* key = field.name.c_str();
			const TypeDescriptor* type_desc = field.find_type_descriptor();
			if (!type_desc)
			{
				return false;
			}

			const TypeId& type = field.type_id;
			if (type == GET_TYPE_ID(int))
				py_dict[key] = blackboard.get<int>(field);
			else if (type == GET_TYPE_ID(float))
				py_dict[key] = blackboard.get<float>(field);
			else if (type == GET_TYPE_ID(double))
				py_dict[key] = blackboard.get<double>(field);
			else if (type == GET_TYPE_ID(bool))
				py_dict[key] = blackboard.get<bool>(field);
			else if (type == GET_TYPE_ID(FixedString8))
				py_dict[key] = blackboard.get<FixedString8>(field).c_str();
			else if (type == GET_TYPE_ID(FixedString16))
				py_dict[key] = blackboard.get<FixedString16>(field).c_str();
			else if (type == GET_TYPE_ID(FixedString32))
				py_dict[key] = blackboard.get<FixedString32>(field).c_str();
			else if (type == GET_TYPE_ID(FixedString64))
				py_dict[key] = blackboard.get<FixedString64>(field).c_str();
			else if (type == GET_TYPE_ID(FixedString128))
				py_dict[key] = blackboard.get<FixedString128>(field).c_str();
			else if (type == GET_TYPE_ID(FixedString256))
				py_dict[key] = blackboard.get<FixedString256>(field).c_str();
			else if (type == GET_TYPE_ID(FixedString512))
				py_dict[key] = blackboard.get<FixedString512>(field).c_str();
			else if (type == GET_TYPE_ID(FixedString1024))
				py_dict[key] = blackboard.get<FixedString1024>(field).c_str();
			else if (type_desc->get_enum_desc() != nullptr)
			{
				char enum_text[128] = {};
				void* field_ptr = blackboard.get(field, type_desc->size);
				if (!field_ptr || !type_desc->to_string(field_ptr, enum_text, sizeof(enum_text)))
				{
					return false;
				}
				py_dict[key] = enum_text;
			}
			else
			{
				return false;
			}

			return true;
		}
	} // namespace

	struct PythonConfig
	{
		FixedString128 script_name;
		FixedString64 class_name;
		Blackboard script;
	};

	struct PythonInputs
	{
		Blackboard script;
	};

	struct PythonOutputs
	{
		Blackboard script;
	};

	struct __attribute__((visibility("hidden"))) PythonInternalState
	{
		py::object py_module;
		py::object py_class;
		py::object py_instance;

		HeapVector<FieldDescriptor> config_fields;
		HeapVector<FieldDescriptor> input_fields;
		HeapVector<FieldDescriptor> output_fields;
		List<FixedString64> string_storage;
	};

	struct __attribute__((visibility("hidden"))) PythonWorkload
	{
		PythonConfig config;
		PythonInputs inputs;
		PythonOutputs outputs;

		StatePtr<PythonInternalState, false> internal_state;
		// ^- EnforceLargeState=false: allow small state while benefitting from ability to explictly destroy it

		PythonWorkload() {}

		~PythonWorkload()
		{
			py::gil_scoped_acquire gil;
			internal_state.destroy(); // explicitly destroy to ensure Python objects are cleaned up while GIL is held
		}

		PythonWorkload(const PythonWorkload&) = delete;
		PythonWorkload& operator=(const PythonWorkload&) = delete;
		PythonWorkload(PythonWorkload&&) noexcept = default;
		PythonWorkload& operator=(PythonWorkload&&) noexcept = default;

		void parse_blackboard_schema(const py::dict& desc_dict, HeapVector<FieldDescriptor>& fields, List<FixedString64>& string_storage)
		{
			size_t field_offset = 0;
			size_t field_index = 0;
			fields.initialize(desc_dict.size());

			for (auto item : desc_dict)
			{
				FieldDescriptor& field_desc = fields[field_index];

				// Extract field name
				const FixedString64 name_str = py_to_fixed_string<FixedString64>(item.first);
				field_desc.name = string_storage.push_back(name_str).c_str(); // safe: copied into FixedString

				// Extract and parse type string
				const FixedString64 original_type_str = py_to_fixed_string<FixedString64>(item.second);
				FixedString64 type_str = original_type_str;
				to_lower(type_str);

				// Set type_id based on known strings
				if (type_str == "int")
					field_desc.type_id = TypeId(GET_TYPE_ID(int));
				else if (type_str == "float")
					field_desc.type_id = TypeId(GET_TYPE_ID(float));
				else if (type_str == "double")
					field_desc.type_id = TypeId(GET_TYPE_ID(double));
				else if (type_str == "bool")
					field_desc.type_id = TypeId(GET_TYPE_ID(bool));
				else if (type_str == "fixedstring8")
					field_desc.type_id = TypeId(GET_TYPE_ID(FixedString8));
				else if (type_str == "fixedstring16")
					field_desc.type_id = TypeId(GET_TYPE_ID(FixedString16));
				else if (type_str == "fixedstring32")
					field_desc.type_id = TypeId(GET_TYPE_ID(FixedString32));
				else if (type_str == "fixedstring64")
					field_desc.type_id = TypeId(GET_TYPE_ID(FixedString64));
				else if (type_str == "fixedstring128")
					field_desc.type_id = TypeId(GET_TYPE_ID(FixedString128));
				else if (type_str == "fixedstring256")
					field_desc.type_id = TypeId(GET_TYPE_ID(FixedString256));
				else if (type_str == "fixedstring512")
					field_desc.type_id = TypeId(GET_TYPE_ID(FixedString512));
				else if (type_str == "fixedstring1024")
					field_desc.type_id = TypeId(GET_TYPE_ID(FixedString1024));
				else
				{
					const TypeDescriptor* custom_type = TypeRegistry::get().find_by_name(original_type_str.c_str());
					if (custom_type && custom_type->get_enum_desc() != nullptr)
					{
						field_desc.type_id = custom_type->id;
					}
					else
					{
						ROBOTICK_FATAL_EXIT("Unsupported field type: %s (resolved from '%s')", type_str.c_str(), original_type_str.c_str());
					}
				}

				// Resolve TypeDescriptor
				const TypeDescriptor* field_type = field_desc.find_type_descriptor();
				if (!field_type)
				{
					ROBOTICK_FATAL_EXIT(
						"Could not find type '%s' for Blackboard field: %s", field_desc.type_id.get_debug_name(), field_desc.name.c_str());
				}

				// Align field_offset based on required alignment
				const size_t align = field_type->alignment;
				field_offset = (field_offset + align - 1) & ~(align - 1);

				field_desc.offset_within_container = field_offset;
				field_offset += field_type->size;
				field_index++;
			}
		}

		void initialize_blackboards(py::object& py_class)
		{
			py::dict desc;

			// (note - we allow exceptions in PythonWorkload/Runtime only since Python libs require them - so the below is fine even with the wider
			// engine not supporting exceptions)
			try
			{
				desc = py_class.attr("describe")();
			}
			catch (const py::error_already_set& e)
			{
				ROBOTICK_FATAL_EXIT("Python class '%s' describe() failed: %s", config.class_name.c_str(), e.what());
			}

			parse_blackboard_schema(desc["config"], internal_state->config_fields, internal_state->string_storage);
			config.script.initialize_fields(internal_state->config_fields);

			parse_blackboard_schema(desc["inputs"], internal_state->input_fields, internal_state->string_storage);
			inputs.script.initialize_fields(internal_state->input_fields);

			parse_blackboard_schema(desc["outputs"], internal_state->output_fields, internal_state->string_storage);
			outputs.script.initialize_fields(internal_state->output_fields);
		}

		void pre_load()
		{
			if (config.script_name.empty() || config.class_name.empty())
				ROBOTICK_FATAL_EXIT("PythonWorkload config must specify script_name and class_name");

			robotick::ensure_python_runtime();
			py::gil_scoped_acquire gil;

			internal_state->py_module = py::module_::import(config.script_name.c_str());
			internal_state->py_class = internal_state->py_module.attr(config.class_name.c_str());

			initialize_blackboards(internal_state->py_class);
		}

		void load()
		{
			robotick::ensure_python_runtime();
			py::gil_scoped_acquire gil;

			const StructDescriptor& struct_desc = config.script.get_struct_descriptor();

			py::dict py_cfg;
			for (size_t i = 0; i < struct_desc.fields.size(); ++i)
			{
				const FieldDescriptor& field = struct_desc.fields[i];
				if (!blackboard_field_to_py_dict(config.script, field, py_cfg))
				{
					ROBOTICK_FATAL_EXIT("Unsupported config field type for key '%s' in PythonWorkload", field.name.c_str());
				}
			}

			// (note - we allow exceptions in PythonWorkload/Runtime only since Python libs require them - so the below is fine even with the wider
			// engine not supporting exceptions)
			try
			{
				internal_state->py_instance = internal_state->py_class(py_cfg);
			}
			catch (py::error_already_set& e)
			{
				// Force error printing to stderr (optional, for developer visibility)
				e.restore(); // Restores the Python error indicator

				// Try to extract richer info manually
				const FixedString256 error_summary(e.what());

				// Get traceback string (if available)
				FixedString1024 traceback_str;
				try
				{
					py::gil_scoped_acquire gil; // Already held here, but safe

					py::object traceback = py::module_::import("traceback");
					py::object tb_list = traceback.attr("format_exception")(e.type(), e.value(), e.trace());

					traceback_str = py_to_fixed_string<FixedString1024>(py::str("").attr("join")(tb_list));
				}
				catch (...)
				{
					traceback_str = "<failed to get Python traceback>";
				}

				ROBOTICK_FATAL_EXIT("Python class '%s' instantiation failed.\n"
									"Exception: %s\n\nTraceback:\n%s",
					config.class_name.c_str(),
					error_summary.c_str(),
					traceback_str.c_str());
			}
		}

		void tick(const TickInfo& tick_info)
		{
			if (!internal_state->py_instance)
				return;

			py::gil_scoped_acquire gil;

			py::dict py_in;
			py::dict py_out;

			const StructDescriptor& struct_desc = inputs.script.get_struct_descriptor();

			for (size_t i = 0; i < struct_desc.fields.size(); ++i)
			{
				const FieldDescriptor& field = struct_desc.fields[i];
				if (!blackboard_field_to_py_dict(inputs.script, field, py_in))
				{
					ROBOTICK_WARNING("Unsupported input field type for key '%s' in PythonWorkload", field.name.c_str());
				}
			}

			// (note - we allow exceptions in PythonWorkload/Runtime only since Python libs require them - so the below is fine even with the wider
			// engine not supporting exceptions)
			try
			{
				internal_state->py_instance.attr("tick")(tick_info.delta_time, py_in, py_out);
			}
			catch (const py::error_already_set& e)
			{
				ROBOTICK_WARNING("Python tick() failed: %s", e.what());
			}

			for (auto item : py_out)
			{
				const FixedString64 key_str = py_to_fixed_string<FixedString64>(item.first); // temporary, for .c_str()
				const char* key = key_str.c_str();
				auto val = item.second;

				const StructDescriptor& output_struct_desc = outputs.script.get_struct_descriptor();
				const FieldDescriptor* found_field = output_struct_desc.find_field(key);

				if (!found_field)
					continue;

				if (found_field->type_id == GET_TYPE_ID(int))
					outputs.script.set<int>(key, val.cast<int>());
				else if (found_field->type_id == GET_TYPE_ID(float))
					outputs.script.set<float>(key, val.cast<float>());
				else if (found_field->type_id == GET_TYPE_ID(double))
					outputs.script.set<double>(key, val.cast<double>());
				else if (found_field->type_id == GET_TYPE_ID(bool))
					outputs.script.set<bool>(key, val.cast<bool>());
				else if (found_field->type_id == GET_TYPE_ID(FixedString8))
					outputs.script.set<FixedString8>(key, py_to_fixed_string<FixedString8>(val));
				else if (found_field->type_id == GET_TYPE_ID(FixedString16))
					outputs.script.set<FixedString16>(key, py_to_fixed_string<FixedString16>(val));
				else if (found_field->type_id == GET_TYPE_ID(FixedString32))
					outputs.script.set<FixedString32>(key, py_to_fixed_string<FixedString32>(val));
				else if (found_field->type_id == GET_TYPE_ID(FixedString64))
					outputs.script.set<FixedString64>(key, py_to_fixed_string<FixedString64>(val));
				else if (found_field->type_id == GET_TYPE_ID(FixedString128))
					outputs.script.set<FixedString128>(key, py_to_fixed_string<FixedString128>(val));
				else if (found_field->type_id == GET_TYPE_ID(FixedString256))
					outputs.script.set<FixedString256>(key, py_to_fixed_string<FixedString256>(val));
				else if (found_field->type_id == GET_TYPE_ID(FixedString512))
					outputs.script.set<FixedString512>(key, py_to_fixed_string<FixedString512>(val));
				else if (found_field->type_id == GET_TYPE_ID(FixedString1024))
					outputs.script.set<FixedString1024>(key, py_to_fixed_string<FixedString1024>(val));
				else
				{
					const TypeDescriptor* field_type = found_field->find_type_descriptor();
					if (field_type && field_type->get_enum_desc() != nullptr)
					{
						const FixedString128 enum_text = py_to_fixed_string<FixedString128>(py::str(val));
						void* output_ptr = outputs.script.get(*found_field, field_type->size);
						if (!output_ptr || !field_type->from_string(enum_text.c_str(), output_ptr))
						{
							ROBOTICK_WARNING("Failed enum marshal for Python output field '%s' from value '%s'", key, enum_text.c_str());
						}
					}
				}
			}
		}
	};

#ifdef ROBOTICK_BUILD_CORE_WORKLOAD_TESTS

	ROBOTICK_REGISTER_STRUCT_BEGIN(PythonConfig)
	ROBOTICK_STRUCT_FIELD(PythonConfig, FixedString128, script_name)
	ROBOTICK_STRUCT_FIELD(PythonConfig, FixedString64, class_name)
	ROBOTICK_STRUCT_FIELD(PythonConfig, Blackboard, script)
	ROBOTICK_REGISTER_STRUCT_END(PythonConfig)

	ROBOTICK_REGISTER_STRUCT_BEGIN(PythonInputs)
	ROBOTICK_STRUCT_FIELD(PythonInputs, Blackboard, script)
	ROBOTICK_REGISTER_STRUCT_END(PythonInputs)

	ROBOTICK_REGISTER_STRUCT_BEGIN(PythonOutputs)
	ROBOTICK_STRUCT_FIELD(PythonOutputs, Blackboard, script)
	ROBOTICK_REGISTER_STRUCT_END(PythonOutputs)

	ROBOTICK_REGISTER_WORKLOAD(PythonWorkload, PythonConfig, PythonInputs, PythonOutputs)

#endif // #ifdef ROBOTICK_BUILD_CORE_WORKLOAD_TESTS

} // namespace robotick

#endif // ROBOTICK_PLATFORM_DESKTOP || ROBOTICK_PLATFORM_LINUX
