// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#include "robotick/systems/audio/WavFile.h"
#include "robotick/api.h"

#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>

namespace robotick
{

	bool WavFile::exists(const char* path)
	{
		std::ifstream f(path, std::ios::binary);
		return (bool)f;
	}

	// ------------------------------------------------------------------------
	// === Load (read-only) ===
	// ------------------------------------------------------------------------
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

		bool have_fmt = false, have_data = false;
		uint16_t audio_format = 0, num_channels = 0, bits_per_sample = 0;
		uint32_t data_size = 0;
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
		this->frame_count = data_size / (num_channels * bytes_per_sample);

		left_samples.initialize(frame_count);
		right_samples.initialize(frame_count);

		for (size_t i = 0; i < frame_count; ++i)
		{
			int16_t l = 0, r = 0;
			f.read(reinterpret_cast<char*>(&l), 2);
			if (num_channels == 2)
				f.read(reinterpret_cast<char*>(&r), 2);
			else
				r = l;
			left_samples[i] = static_cast<float>(l) / 32768.0f;
			right_samples[i] = static_cast<float>(r) / 32768.0f;
		}

		ROBOTICK_INFO("WAV loaded: %s (%zu frames, %u Hz, %u-bit, %u ch)", path, frame_count, sample_rate, bits_per_sample, num_channels);
		return true;
	}

	float WavFile::get_duration_seconds() const
	{
		return sample_rate > 0 ? static_cast<float>(get_frame_count()) / sample_rate : 0.0f;
	}

	// ------------------------------------------------------------------------
	// === Write (recording) ===
	// ------------------------------------------------------------------------

	void WavFile::write_header_placeholder(uint32_t sr, uint16_t ch)
	{
		const uint16_t bits_per_sample = 16;
		const uint32_t byte_rate = sr * ch * bits_per_sample / 8;
		const uint16_t block_align = ch * bits_per_sample / 8;

		std::fwrite("RIFF", 1, 4, fp);
		uint32_t riff_size_placeholder = 0;
		std::fwrite(&riff_size_placeholder, 4, 1, fp);
		std::fwrite("WAVE", 1, 4, fp);

		std::fwrite("fmt ", 1, 4, fp);
		uint32_t fmt_size = 16;
		std::fwrite(&fmt_size, 4, 1, fp);
		uint16_t format_tag = 1;
		std::fwrite(&format_tag, 2, 1, fp);
		std::fwrite(&ch, 2, 1, fp);
		std::fwrite(&sr, 4, 1, fp);
		std::fwrite(&byte_rate, 4, 1, fp);
		std::fwrite(&block_align, 2, 1, fp);
		std::fwrite(&bits_per_sample, 2, 1, fp);

		std::fwrite("data", 1, 4, fp);
		uint32_t data_size_placeholder = 0;
		std::fwrite(&data_size_placeholder, 4, 1, fp);
	}

	bool WavFile::open_write(const char* path, uint32_t sr, uint16_t ch)
	{
		fp = std::fopen(path, "wb");
		if (!fp)
		{
			ROBOTICK_WARNING("Failed to open WAV for writing: %s", path);
			return false;
		}
		sample_rate = sr;
		write_channels = ch;
		data_bytes_written = 0;
		write_header_placeholder(sr, ch);
		return true;
	}

	void WavFile::patch_header()
	{
		if (!fp)
			return;

		// Save current position
		long pos = std::ftell(fp);

		const uint32_t riff_size = data_bytes_written + 36;
		const uint32_t data_size = data_bytes_written;

		std::fflush(fp);
		std::fseek(fp, 4, SEEK_SET);
		std::fwrite(&riff_size, 4, 1, fp);
		std::fseek(fp, 40, SEEK_SET);
		std::fwrite(&data_size, 4, 1, fp);
		std::fflush(fp);

		// Restore original position
		if (pos >= 0)
		{
			std::fseek(fp, pos, SEEK_SET);
		}
	}

	void WavFile::append_mono(const float* samples, size_t count)
	{
		if (!fp || write_channels != 1 || count == 0)
			return;

		for (size_t i = 0; i < count; ++i)
		{
			float v = std::clamp(samples[i], -1.0f, 1.0f);
			int16_t s16 = static_cast<int16_t>(std::round(v * 32767.0f));
			std::fwrite(&s16, sizeof(int16_t), 1, fp);
			data_bytes_written += 2;
		}

		patch_header(); // keep file valid even on crash
	}

	void WavFile::append_stereo(const float* left, const float* right, size_t count)
	{
		if (!fp || write_channels != 2 || count == 0)
			return;

		for (size_t i = 0; i < count; ++i)
		{
			float l = std::clamp(left[i], -1.0f, 1.0f);
			float r = std::clamp(right[i], -1.0f, 1.0f);
			int16_t sL = static_cast<int16_t>(std::round(l * 32767.0f));
			int16_t sR = static_cast<int16_t>(std::round(r * 32767.0f));
			std::fwrite(&sL, sizeof(int16_t), 1, fp);
			std::fwrite(&sR, sizeof(int16_t), 1, fp);
			data_bytes_written += 4;
		}

		patch_header(); // always keep header current
	}

	void WavFile::close_write()
	{
		if (!fp)
			return;
		patch_header();
		std::fclose(fp);
		fp = nullptr;
		write_channels = 0;
		data_bytes_written = 0;
	}

} // namespace robotick
