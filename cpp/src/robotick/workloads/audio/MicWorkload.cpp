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
		float amplitude_gain_db = 0.0f; // Linear gain multiplier = pow(10, amplitude_gain_db / 20)
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

			const float gain_db = config.amplitude_gain_db;
			if (std::fabs(gain_db) > 1e-6f)
			{
				const float gain = std::pow(10.0f, gain_db / 20.0f);
				for (size_t i = 0; i < num_samples_read; ++i)
				{
					outputs.mono.samples[i] *= gain;
				}
			}
		}
	};

} // namespace robotick
