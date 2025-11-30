// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#include "robotick/api.h"
#include "robotick/systems/audio/AudioFrame.h"
#include "robotick/systems/audio/AudioSystem.h"

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
		AudioQueueResult last_read_status = AudioQueueResult::Success;
		uint32_t dropped_reads = 0;
	};

	struct MicWorkload
	{
		MicConfig config;
		MicOutputs outputs;

		// One-time bring-up. Safe to call multiple times if the engine does.
		void load()
		{
			AudioSystem::init();
			const uint32_t input_rate = AudioSystem::get_input_sample_rate();
			outputs.mono.sample_rate = (input_rate != 0) ? input_rate : AudioSystem::get_sample_rate();
		}

		// Pull a chunk from the mic and publish to outputs.
		void tick(const TickInfo& tick_info)
		{
			static constexpr double ns_to_sec = 1e-9;
			outputs.mono.timestamp = ns_to_sec * (double)tick_info.time_now_ns;

			// Read up to the buffer capacity from the mic.
			const AudioReadResult read_result = AudioSystem::read(outputs.mono.samples.data(), outputs.mono.samples.capacity());
			outputs.last_read_status = read_result.status;
			outputs.mono.samples.set_size(read_result.samples_read);

			if (read_result.status == AudioQueueResult::Dropped)
			{
				// Queue empty; surface telemetry and keep output empty for this tick
				outputs.dropped_reads++;
				return;
			}
			if (read_result.status == AudioQueueResult::Error)
			{
				ROBOTICK_WARNING("MicWorkload failed to read from AudioSystem input");
				outputs.dropped_reads++;
				outputs.mono.samples.set_size(0);
				return;
			}

			const size_t num_samples_read = read_result.samples_read;

			const float gain_db = config.amplitude_gain_db;
			if (fabsf(gain_db) > 1e-6f)
			{
				const float gain = powf(10.0f, gain_db / 20.0f);
				for (size_t i = 0; i < num_samples_read; ++i)
				{
					outputs.mono.samples[i] *= gain;
				}
			}
		}
	};

} // namespace robotick
