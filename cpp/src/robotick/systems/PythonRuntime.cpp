// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#if defined(ROBOTICK_PLATFORM_DESKTOP) || defined(ROBOTICK_PLATFORM_LINUX)

#include "robotick/systems/PythonRuntime.h"
#include "robotick/config/AssertUtils.h"

#include <pybind11/embed.h>

namespace py = pybind11;

namespace robotick
{
	namespace
	{
		PythonRuntimeConfig g_runtime_config{};
		bool g_custom_config = false;
		bool g_runtime_initialized = false;

		void append_module_paths(const PythonRuntimeConfig& config)
		{
			if (!config.extra_module_paths || config.extra_module_path_count == 0)
				return;

			py::object sys = py::module::import("sys");
			py::object path = sys.attr("path");
			for (size_t i = 0; i < config.extra_module_path_count; ++i)
			{
				const char* entry = config.extra_module_paths[i];
				if (!entry || entry[0] == '\0')
					continue;
				path.attr("append")(entry);
			}
		}
	} // namespace

	void set_python_runtime_config(const PythonRuntimeConfig& config)
	{
		if (g_runtime_initialized)
			ROBOTICK_FATAL_EXIT("Python runtime already initialized; cannot change configuration");
		g_runtime_config = config;
		g_custom_config = true;
	}

	const PythonRuntimeConfig& get_python_runtime_config()
	{
		return g_runtime_config;
	}

	bool python_runtime_is_initialized()
	{
		return g_runtime_initialized;
	}

	void ensure_python_runtime()
	{
		if (g_runtime_initialized)
			return;

		if (!g_custom_config)
			g_runtime_config = PythonRuntimeConfig{};

		if (!g_runtime_config.import_site)
		{
#ifdef Py_NoSiteFlag
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
			Py_NoSiteFlag = 1;
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
#endif
		}
#ifdef Py_NoUserSiteDirectory
		if (!g_runtime_config.allow_user_site)
		{
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
			Py_NoUserSiteDirectory = 1;
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
		}
#endif

		static auto* guard = []()
		{
			auto* interpreter = new py::scoped_interpreter{};
			append_module_paths(g_runtime_config);
			if (g_runtime_config.post_init_hook)
				g_runtime_config.post_init_hook();
			PyEval_SaveThread();
			return interpreter;
		}();
		(void)guard;
		g_runtime_initialized = true;
	}

} // namespace robotick

#else

#include "robotick/systems/PythonRuntime.h"

namespace robotick
{
	namespace
	{
		PythonRuntimeConfig g_runtime_config{};
	} // namespace

	void set_python_runtime_config(const PythonRuntimeConfig& config)
	{
		g_runtime_config = config;
	}

	const PythonRuntimeConfig& get_python_runtime_config()
	{
		return g_runtime_config;
	}

	bool python_runtime_is_initialized()
	{
		return false;
	}

	inline void ensure_python_runtime()
	{
	}
} // namespace robotick

#endif
