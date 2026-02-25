// Copyright Robotick contributors
// SPDX-License-Identifier: Apache-2.0

#include "robotick/api.h"

namespace robotick
{
	/// StubWorkload: A minimal no-op workload used as a stand-in
	/// during development. Replace with concrete workload implementations
	/// when functionality is ready.

	struct StubWorkload
	{
		void tick(const TickInfo& info) { (void)info; }
	};

} // namespace robotick
