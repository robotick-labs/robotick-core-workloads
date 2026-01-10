// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "robotick/api.h"
#include "robotick/framework/concurrency/Atomic.h"

#include <mujoco/mujoco.h>

namespace robotick
{
	namespace mujoco_callbacks
	{
		inline void user_warning(const char* message)
		{
			ROBOTICK_WARNING("MuJoCo warning: %s", message ? message : "");
		}

		inline void user_error(const char* message)
		{
			ROBOTICK_WARNING("MuJoCo error: %s", message ? message : "");
		}

		inline void install()
		{
			static AtomicFlag installed(false);
			if (installed.test_and_set())
				return;

			mju_user_warning = user_warning;
			mju_user_error = user_error;
		}
	} // namespace mujoco_callbacks
} // namespace robotick
