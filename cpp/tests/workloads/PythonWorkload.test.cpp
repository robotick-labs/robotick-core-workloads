// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#include "robotick/api.h"
#include "robotick/framework/Engine.h"
#include "robotick/framework/data/Blackboard.h"
#include "robotick/framework/utils/TypeId.h"
#include "robotick/systems/PythonRuntime.h"

#include <cstdlib>
#include <cstring>

#include <catch2/catch_all.hpp>

namespace robotick
{
	void ensure_python_workload()
	{
		ROBOTICK_KEEP_WORKLOAD(PythonWorkload)
	}

} // namespace robotick

using namespace robotick;

namespace
{
	static FixedString1024 g_python_module_path;
	static const char* g_python_path_array[1] = {nullptr};
	static bool g_runtime_configured = false;

	static void trim_to_parent(FixedString1024& path)
	{
		char* data = path.str();
		size_t len = path.length();
		while (len > 0)
		{
			--len;
			if (data[len] == '/' || data[len] == '\\')
			{
				data[len] = '\0';
				return;
			}
			data[len] = '\0';
		}
	}

	static FixedString1024 compute_python_path()
	{
		// Derive the path to the test python package by walking up from this
		// test source file (…/tests/workloads/) to the repository root and
		// then appending the fixed "python" folder. This mirrors the repo
		// layout so tests can set PYTHONPATH without hard‑coding absolute
		// paths that would break on other machines or CI agents.
		FixedString1024 result(__FILE__);
		trim_to_parent(result); // remove filename
		for (int i = 0; i < 3; ++i)
			trim_to_parent(result);
		result.append("/python");
		return result;
	}
} // namespace

#ifndef ROBOTICK_ENABLE_PYTHON
#error "ROBOTICK_ENABLE_PYTHON must be defined (expected value: 1)"
#endif

#if ROBOTICK_ENABLE_PYTHON != 1
#error "ROBOTICK_ENABLE_PYTHON must be set to 1"
#endif

TEST_CASE("Unit/Workloads/PythonWorkload")
{
	// ============================
	// Setup for Tests:
	// ============================
	ensure_python_workload(); // ensure it doesn't get deadstripped

	// ensure we can access our hello_workload.py as a Python module:
	{
		g_python_module_path = compute_python_path();
		g_python_path_array[0] = g_python_module_path.c_str();
		setenv("PYTHONPATH", g_python_module_path.c_str(), 1);
		ROBOTICK_INFO("🧪 PYTHONPATH set for test: %s", g_python_module_path.c_str());
		if (!g_runtime_configured && !python_runtime_is_initialized())
		{
			PythonRuntimeConfig runtime_config;
			runtime_config.extra_module_paths = g_python_path_array;
			runtime_config.extra_module_path_count = 1;
			set_python_runtime_config(runtime_config);
			g_runtime_configured = true;
		}
	}

	// ============================
	// 🧪 Test Sections
	// ============================

	SECTION("Python tick executes")
	{
		Model model;
		static const FieldConfigEntry python_config[] = {
			{"script_name", "robotick.workloads.optional.test.hello_workload"},
			{"class_name", "HelloWorkload"}
		};
		static const WorkloadSeed python_workload{
			TypeId("PythonWorkload"),
			StringView("test2"),
			1.0f,
			{},
			python_config,
			{}
		};
		static const WorkloadSeed* workloads[] = {&python_workload};
		model.use_workload_seeds(workloads);
		model.set_root_workload(python_workload);

		Engine engine;
		engine.load(model);
		const auto& info = *engine.find_instance_info(python_workload.unique_name);
		auto* inst_ptr = info.get_ptr(engine);

		REQUIRE(inst_ptr);
		REQUIRE(info.type != nullptr);
		REQUIRE(info.type->get_workload_desc() != nullptr);
		REQUIRE(info.type->get_workload_desc()->tick_fn != nullptr);

		REQUIRE_NOTHROW(info.type->get_workload_desc()->tick_fn(inst_ptr, TICK_INFO_FIRST_10MS_100HZ));
	}

	SECTION("Output reflects Python computation")
	{
		Model model;
		static const FieldConfigEntry python_config[] = {
			{"script_name", "robotick.workloads.optional.test.hello_workload"},
			{"class_name", "HelloWorkload"},
			{"script.example_in", "21.0"}
		};
		static const WorkloadSeed root{
			TypeId("PythonWorkload"),
			StringView("py"),
			1.0f,
			{},
			python_config,
			{}
		};
		static const WorkloadSeed* workloads[] = {&root};
		model.use_workload_seeds(workloads);
		model.set_root_workload(root);

		Engine engine;
		engine.load(model);

		const auto& info = *engine.find_instance_info(root.unique_name);
		auto* inst_ptr = info.get_ptr(engine);
		REQUIRE(inst_ptr != nullptr);
		REQUIRE(info.type != nullptr);
		REQUIRE(info.type->get_workload_desc()->tick_fn != nullptr);

		// Execute tick
		info.type->get_workload_desc()->tick_fn(inst_ptr, TICK_INFO_FIRST_10MS_100HZ);

		// === Find the output blackboard ===
		const auto* outputs_desc = info.type->get_workload_desc()->outputs_desc;
		const size_t outputs_offset = info.type->get_workload_desc()->outputs_offset;
		REQUIRE(outputs_desc != nullptr);
		REQUIRE(outputs_offset != OFFSET_UNBOUND);

		const void* output_base = static_cast<const uint8_t*>(inst_ptr) + outputs_offset;

		const robotick::Blackboard* output_blackboard = nullptr;
		for (const auto& field : outputs_desc->get_struct_desc()->fields)
		{
			if (field.name == "script")
			{
				ROBOTICK_ASSERT(field.offset_within_container != OFFSET_UNBOUND && "Field offset should have been correctly set by now");

				const void* field_ptr = static_cast<const uint8_t*>(output_base) + field.offset_within_container;
				output_blackboard = static_cast<const robotick::Blackboard*>(field_ptr);
				break;
			}
		}

		REQUIRE(output_blackboard->has("greeting"));
		const FixedString64 greeting = output_blackboard->get<FixedString64>("greeting");
		REQUIRE(::strncmp(greeting.c_str(), "[Python] Hello!", 15) == 0);

		REQUIRE(output_blackboard->has("val_double"));
		const double val_double = output_blackboard->get<double>("val_double");
		REQUIRE(val_double == 1.23);

		REQUIRE(output_blackboard->has("val_int"));
		const int val_int = output_blackboard->get<int>("val_int");
		REQUIRE(val_int == 456);
	}

	SECTION("start/stop hooks are optional and safe")
	{
		Model model;
		static const FieldConfigEntry python_config[] = {
			{"script_name", "robotick.workloads.optional.test.hello_workload"},
			{"class_name", "HelloWorkload"}
		};
		static const WorkloadSeed root{
			TypeId("PythonWorkload"),
			StringView("test"),
			10.0f,
			{},
			python_config,
			{}
		};
		static const WorkloadSeed* workloads[] = {&root};
		model.use_workload_seeds(workloads);
		model.set_root_workload(root);

		Engine engine;
		engine.load(model);

		const auto& info = *engine.find_instance_info(root.unique_name);
		auto* inst_ptr = info.get_ptr(engine);

		REQUIRE(inst_ptr != nullptr);
		REQUIRE(info.type != nullptr);

		if (info.type->get_workload_desc()->start_fn)
		{
			REQUIRE_NOTHROW(info.type->get_workload_desc()->start_fn(inst_ptr, 10.0f));
		}
		if (info.type->get_workload_desc()->stop_fn)
		{
			REQUIRE_NOTHROW(info.type->get_workload_desc()->stop_fn(inst_ptr));
		}
	}
}
