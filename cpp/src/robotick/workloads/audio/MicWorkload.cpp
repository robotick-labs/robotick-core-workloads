// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "robotick/api.h"

#include "robotick/systems/audio/AudioFrame.h"
#include "robotick/systems/audio/AudioSystem.h"

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace robotick
{
	// ===============================================
	// === MicWorkload ===============================
	// ===============================================

	// Tunables kept tiny on purpose; adjust as needed.
	struct MicConfig
	{
		float gain = 1.0f; // linear gain applied to captured samples
	};

	struct MicOutputs
	{
		AudioFrame mono; // most recent captured block (mono, float32)
	};

	struct MicWorkload
	{
		MicConfig config;
		MicOutputs outputs;

		// One-time bring-up. Safe to call multiple times if the engine does.
		void load()
		{
			AudioSystem::init();
			outputs.mono.sample_rate = AudioSystem::get_sample_rate(); // constant once init'd
		}

		// Pull a chunk from the mic and publish to outputs.
		void tick(const TickInfo& tick_info)
		{
			static constexpr double ns_to_sec = 1e-9;
			outputs.mono.timestamp = ns_to_sec * (double)tick_info.time_now_ns;

			// Read up to the buffer capacity from the mic.
			const size_t num_samples_read = AudioSystem::read(outputs.mono.samples.data(), outputs.mono.samples.capacity());
			outputs.mono.samples.set_size(num_samples_read);
		}
	};

} // namespace robotick
