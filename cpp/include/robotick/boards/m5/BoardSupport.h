// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#pragma once

namespace robotick::boards::m5
{
	/// Ensure the global M5 runtime is initialized.
	/// Returns true when the board-specific init succeeded (or was already initialized).
	/// Returns false when the helper is unavailable on the current platform.
	bool ensure_initialized();
} // namespace robotick::boards::m5
