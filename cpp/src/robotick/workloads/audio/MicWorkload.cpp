// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "robotick/api.h"

#include "robotick/systems/audio/AudioBuffer.h"
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
		float gain = 1.0f;			 // linear gain applied to captured samples
		float vad_threshold = 0.01f; // RMS threshold for simple VAD (tweak per setup)
	};

	struct MicOutputs
	{
		AudioBuffer512 mono; // most recent captured block (mono, float32)
		float rms = 0.0f;	 // RMS of the emitted block (post-gain)
		bool voice = false;	 // simple VAD flag (rms >= vad_threshold)
	};

	struct MicWorkload
	{
		MicConfig config;
		MicOutputs outputs;

		// One-time bring-up. Safe to call multiple times if the engine does.
		void load() { AudioSystem::init(); }

		// Pull a chunk from the mic and publish to outputs.
		void tick(const TickInfo&)
		{
			// Read up to the buffer capacity from the mic.
			float tmp[512];
			const size_t got = AudioSystem::read(tmp, 512);
			if (got == 0)
			{
				// No new audio — clear outputs for clarity.
				// Assuming AudioBuffer512 exposes set_size(0).
				outputs.mono.set_size(0);
				outputs.rms = 0.0f;
				outputs.voice = false;
				return;
			}

			// Resize the output buffer and copy with gain.
			outputs.mono.set_size(got);

			double sum_sq = 0.0;
			for (size_t i = 0; i < got; ++i)
			{
				const float s = tmp[i] * config.gain;
				outputs.mono[i] = s;
				sum_sq += static_cast<double>(s) * static_cast<double>(s);
			}

			// Compute RMS and a simple VAD flag.
			const double mean_sq = sum_sq / static_cast<double>(got);
			const float rms = mean_sq > 0.0 ? static_cast<float>(std::sqrt(mean_sq)) : 0.0f;
			outputs.rms = rms;
			outputs.voice = (rms >= config.vad_threshold);
		}
	};

} // namespace robotick
