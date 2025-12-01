// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#if defined(ROBOTICK_PLATFORM_DESKTOP) || defined(ROBOTICK_PLATFORM_LINUX)

#include "robotick/systems/PythonRuntime.h"

#include <catch2/catch_all.hpp>
#include <cstring>
#include <pybind11/embed.h>

namespace
{
	bool g_post_init_called = false;
	void test_post_init_hook()
	{
		g_post_init_called = true;
	}

	const char kCustomPath[] = "/tmp/robotick_python_runtime_test";
	const char* const kExtraPaths[] = {kCustomPath};
} // namespace

TEST_CASE("Unit/Systems/PythonRuntime/ConfigurableInit")
{
#if !ROBOTICK_ENABLE_PYTHON
	SUCCEED("Python disabled in this build");
	return;
#endif

	using namespace robotick;

	if (python_runtime_is_initialized())
	{
		SUCCEED("Python runtime already initialized by another test");
		return;
	}

	PythonRuntimeConfig config;
	config.import_site = false;
	config.allow_user_site = false;
	config.extra_module_paths = kExtraPaths;
	config.extra_module_path_count = 1;
	config.post_init_hook = &test_post_init_hook;

	set_python_runtime_config(config);
	ensure_python_runtime();

	REQUIRE(python_runtime_is_initialized());
	CHECK(g_post_init_called);

	pybind11::gil_scoped_acquire gil;
	pybind11::object sys = pybind11::module::import("sys");
	pybind11::list path_entries = sys.attr("path");
	bool found = false;
	for (auto entry : path_entries)
	{
		const char* utf8_value = PyUnicode_AsUTF8(entry.ptr());
		if (utf8_value && ::strcmp(utf8_value, kCustomPath) == 0)
		{
			found = true;
			break;
		}
	}
	CHECK(found);
}

#endif
