// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#if defined(ROBOTICK_PLATFORM_DESKTOP) || defined(ROBOTICK_PLATFORM_LINUX)

#include <pybind11/embed.h>

namespace robotick
{

	void ensure_python_runtime()
	{
		static auto* guard = []()
		{
			auto* g = new pybind11::scoped_interpreter{};
			PyEval_SaveThread();
			return g;
		}();
		(void)guard;
	}

} // namespace robotick

#else

namespace robotick
{
	inline void ensure_python_runtime()
	{
	}
} // namespace robotick

#endif
