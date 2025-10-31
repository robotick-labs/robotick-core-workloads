// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#include "robotick/api.h"
#include "robotick/systems/AudioBuffer.h"
#include "robotick/systems/AudioSystem.h"

#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

namespace robotick
{
	struct WavPlayerConfig
	{
		FixedString256 file_path; // Path to WAV file (16-bit PCM, stereo)

		bool looping = false;
	};

	struct WavPlayerOutputs
	{
		AudioBuffer64 left;
		AudioBuffer64 right;

		uint64_t total_num_frames = 0;
	};

	struct WavPlayerState
	{
		std::vector<float> left_samples;
		std::vector<float> right_samples;
		size_t current_frame = 0;
		int sample_rate = 44100;
	};

	struct WavPlayerWorkload
	{
		WavPlayerConfig config;
		WavPlayerOutputs outputs;
		State<WavPlayerState> state;

		void load()
		{
			AudioSystem::init();

			std::ifstream f(config.file_path.c_str(), std::ios::binary);
			if (!f)
				ROBOTICK_FATAL_EXIT("Failed to open WAV file: %s", config.file_path.c_str());

			// Report file size
			f.seekg(0, std::ios::end);
			const size_t file_size = static_cast<size_t>(f.tellg());
			f.seekg(0);
			ROBOTICK_INFO("Opened WAV file '%s' (%zu bytes)", config.file_path.c_str(), file_size);

			auto read_u32le = [&](uint32_t& out)
			{
				unsigned char b[4];
				f.read(reinterpret_cast<char*>(b), 4);
				out = (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
			};
			auto read_u16le = [&](uint16_t& out)
			{
				unsigned char b[2];
				f.read(reinterpret_cast<char*>(b), 2);
				out = (uint16_t)b[0] | ((uint16_t)b[1] << 8);
			};

			// ===== RIFF header (12 bytes) =====
			char riff_id[4];
			f.read(riff_id, 4);
			uint32_t riff_size = 0;
			read_u32le(riff_size);
			char wave_id[4];
			f.read(wave_id, 4);
			if (std::strncmp(riff_id, "RIFF", 4) != 0 || std::strncmp(wave_id, "WAVE", 4) != 0)
				ROBOTICK_FATAL_EXIT("Not a RIFF/WAVE file: %s", config.file_path.c_str());

			// ===== Scan chunks until we find "fmt " and "data" =====
			bool have_fmt = false;
			bool have_data = false;

			uint16_t audio_format = 0;
			uint16_t num_channels = 0;
			uint32_t sample_rate = 0;
			uint16_t bits_per_sample = 0;

			uint32_t data_size = 0;
			std::streampos data_pos{};

			while (f && (!have_fmt || !have_data))
			{
				char chunk_id[4];
				uint32_t chunk_size = 0;

				f.read(chunk_id, 4);
				if (!f)
					break; // EOF or read error
				read_u32le(chunk_size);

				if (std::strncmp(chunk_id, "fmt ", 4) == 0)
				{
					// Parse the 'fmt ' chunk (at least 16 bytes for PCM)
					read_u16le(audio_format); // PCM = 1
					read_u16le(num_channels); // 1 or 2
					read_u32le(sample_rate);  // e.g. 44100

					uint32_t byte_rate = 0;
					read_u32le(byte_rate);
					uint16_t block_align = 0;
					read_u16le(block_align);
					read_u16le(bits_per_sample); // e.g. 16

					// Skip any extra fmt bytes beyond the standard 16
					const int consumed = 16;
					if (chunk_size > consumed)
						f.seekg(chunk_size - consumed, std::ios::cur);

					have_fmt = true;
				}
				else if (std::strncmp(chunk_id, "data", 4) == 0)
				{
					data_pos = f.tellg();
					data_size = chunk_size;
					// Position stays at start of PCM data (we'll read it below)
					have_data = true;
				}
				else
				{
					// Skip unknown chunk (align to even)
					f.seekg(chunk_size, std::ios::cur);
				}

				// Chunks are word-aligned (pad byte if size is odd)
				if (chunk_size & 1)
					f.seekg(1, std::ios::cur);
			}

			if (!have_fmt || !have_data)
				ROBOTICK_FATAL_EXIT("WAV missing required 'fmt ' or 'data' chunk: %s", config.file_path.c_str());

			if (audio_format != 1 || (bits_per_sample != 16))
				ROBOTICK_FATAL_EXIT("Only 16-bit PCM WAV supported. format=%u, bps=%u", audio_format, bits_per_sample);

			if (num_channels != 1 && num_channels != 2)
				ROBOTICK_FATAL_EXIT("Only mono or stereo WAV supported. channels=%u", num_channels);

			f.seekg(data_pos);

			const size_t bytes_per_sample = bits_per_sample / 8; // 2
			const size_t bytes_per_frame = bytes_per_sample * num_channels;
			const size_t frame_count = data_size / bytes_per_frame;

			state->left_samples.clear();
			state->right_samples.clear();
			state->left_samples.reserve(frame_count);
			state->right_samples.reserve(frame_count);
			state->sample_rate = static_cast<int>(sample_rate);

			for (size_t i = 0; i < frame_count; ++i)
			{
				int16_t l = 0, r = 0;
				if (num_channels == 1)
				{
					// Read mono sample and duplicate to both channels
					f.read(reinterpret_cast<char*>(&l), 2);
					r = l;
				}
				else
				{
					f.read(reinterpret_cast<char*>(&l), 2);
					f.read(reinterpret_cast<char*>(&r), 2);
				}
				state->left_samples.push_back(static_cast<float>(l) / 32768.0f);
				state->right_samples.push_back(static_cast<float>(r) / 32768.0f);
			}

			outputs.total_num_frames = frame_count;

			ROBOTICK_INFO(
				"WAV: %u Hz, %u ch, %u-bit, frames=%zu (data=%u bytes)", sample_rate, num_channels, bits_per_sample, frame_count, data_size);
		}

		void start(float tick_rate_hz)
		{
			// nothing needed yet
		}

		void tick(const TickInfo& info)
		{
			int target_rate = AudioSystem::get_sample_rate();
			int samples_per_tick = target_rate / static_cast<int>(info.tick_rate_hz);

			outputs.left.set_size(0);
			outputs.right.set_size(0);

			for (int i = 0; i < samples_per_tick; ++i)
			{
				if (state->current_frame >= state->left_samples.size())
				{
					if (config.looping)
					{
						state->current_frame = 0;
					}
					else
					{
						break;
					}
				}

				outputs.left.add(state->left_samples[state->current_frame]);
				outputs.right.add(state->right_samples[state->current_frame]);
				state->current_frame++;
			}
		}
	};

} // namespace robotick