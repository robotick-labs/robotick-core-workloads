// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#include "robotick/boards/m5/BoardSupport.h"

#include "robotick/framework/concurrency/Atomic.h"

#if defined(ROBOTICK_PLATFORM_ESP32S3) && defined(ROBOTICK_PLATFORM_ESP32S3_M5)
#include <M5Unified.h>
#define ROBOTICK_M5_SUPPORT 1
#else
#define ROBOTICK_M5_SUPPORT 0
#endif

namespace robotick::boards::m5
{
	bool ensure_initialized()
	{
#if ROBOTICK_M5_SUPPORT
		static AtomicFlag initialized{false};
		if (!initialized.test_and_set())
		{
			M5.begin();
		}
		return true;
#else
		return false;
#endif
	}
} // namespace robotick::boards::m5
