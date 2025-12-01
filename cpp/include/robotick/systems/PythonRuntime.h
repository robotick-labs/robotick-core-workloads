// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstddef>

namespace robotick
{
	struct PythonRuntimeConfig
	{
		bool import_site = true;
		bool allow_user_site = true;
		const char* const* extra_module_paths = nullptr;
		size_t extra_module_path_count = 0;
		void (*post_init_hook)() = nullptr;
	};

	void set_python_runtime_config(const PythonRuntimeConfig& config);
	const PythonRuntimeConfig& get_python_runtime_config();
	bool python_runtime_is_initialized();
	void ensure_python_runtime();
} // namespace robotick
