// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#include "robotick/systems/audio/WavFile.h"

#include "robotick/api.h"

#include <cstring>
#include <fstream>

namespace robotick
{

	bool WavFile::load(const char* path)
	{
		std::ifstream f(path, std::ios::binary);
		if (!f)
		{
			ROBOTICK_WARNING("Failed to open WAV file: %s", path);
			return false;
		}

		// Little-endian helpers
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

		// RIFF header
		char riff_id[4];
		f.read(riff_id, 4);
		uint32_t riff_size;
		read_u32le(riff_size);
		char wave_id[4];
		f.read(wave_id, 4);
		if (std::strncmp(riff_id, "RIFF", 4) != 0 || std::strncmp(wave_id, "WAVE", 4) != 0)
		{
			ROBOTICK_WARNING("Not a RIFF/WAVE file: %s", path);
			return false;
		}

		// Chunk scanning
		bool have_fmt = false, have_data = false;
		uint16_t audio_format = 0, num_channels = 0, bits_per_sample = 0;
		uint32_t sample_rate = 0, data_size = 0;
		std::streampos data_pos;

		while (f && (!have_fmt || !have_data))
		{
			char chunk_id[4];
			uint32_t chunk_size;
			f.read(chunk_id, 4);
			if (!f)
				break;
			read_u32le(chunk_size);

			if (std::strncmp(chunk_id, "fmt ", 4) == 0)
			{
				read_u16le(audio_format);
				read_u16le(num_channels);
				read_u32le(sample_rate);
				uint32_t byte_rate;
				read_u32le(byte_rate);
				uint16_t block_align;
				read_u16le(block_align);
				read_u16le(bits_per_sample);
				if (chunk_size > 16)
					f.seekg(chunk_size - 16, std::ios::cur);
				have_fmt = true;
			}
			else if (std::strncmp(chunk_id, "data", 4) == 0)
			{
				data_pos = f.tellg();
				data_size = chunk_size;
				have_data = true;
			}
			else
			{
				f.seekg(chunk_size, std::ios::cur);
			}

			if (chunk_size & 1)
				f.seekg(1, std::ios::cur);
		}

		if (!have_fmt || !have_data || audio_format != 1 || (bits_per_sample != 16) || (num_channels != 1 && num_channels != 2))
		{
			ROBOTICK_WARNING("Unsupported WAV format in %s", path);
			return false;
		}

		f.seekg(data_pos);
		size_t bytes_per_sample = bits_per_sample / 8;
		size_t frame_count = data_size / (num_channels * bytes_per_sample);

		left_samples.initialize(frame_count);
		right_samples.initialize(frame_count);

		sampleRate = static_cast<int>(sample_rate);

		for (size_t i = 0; i < frame_count; ++i)
		{
			int16_t l = 0, r = 0;
			f.read(reinterpret_cast<char*>(&l), 2);
			if (num_channels == 2)
				f.read(reinterpret_cast<char*>(&r), 2);
			else
				r = l;
			left_samples[i] = (static_cast<float>(l) / 32768.0f);
			right_samples[i] = (static_cast<float>(r) / 32768.0f);
		}

		ROBOTICK_INFO("WAV loaded: %s (%zu frames, %u Hz, %u-bit, %u ch)", path, frame_count, sample_rate, bits_per_sample, num_channels);
		return true;
	}

	float WavFile::get_duration_seconds() const
	{
		return sampleRate > 0 ? static_cast<float>(get_frame_count()) / sampleRate : 0.0f;
	}

} // namespace robotick
