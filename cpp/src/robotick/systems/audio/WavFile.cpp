// Copyright Robotick Labs
// SPDX-License-Identifier: Apache-2.0

#include "robotick/systems/audio/WavFile.h"
#include "robotick/api.h"
#include "robotick/framework/math/MathUtils.h"

#include <cstring>

namespace robotick
{
	namespace
	{
		bool read_u32le(FILE* f, uint32_t& out)
		{
			unsigned char b[4];
			if (::fread(b, 1, 4, f) != 4)
				return false;
			out = (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
			return true;
		}

		bool read_u16le(FILE* f, uint16_t& out)
		{
			unsigned char b[2];
			if (::fread(b, 1, 2, f) != 2)
				return false;
			out = (uint16_t)b[0] | ((uint16_t)b[1] << 8);
			return true;
		}
	} // namespace

	bool WavFile::exists(const char* path)
	{
		FILE* f = ::fopen(path, "rb");
		if (!f)
			return false;
		::fclose(f);
		return true;
	}

	bool WavFile::load(const char* path)
	{
		FILE* f = ::fopen(path, "rb");
		if (!f)
		{
			ROBOTICK_WARNING("Failed to open WAV file: %s", path);
			return false;
		}

		char riff_id[4];
		uint32_t riff_size = 0;
		char wave_id[4];

		if (::fread(riff_id, 1, 4, f) != 4 || !read_u32le(f, riff_size) || ::fread(wave_id, 1, 4, f) != 4)
		{
			ROBOTICK_WARNING("Truncated or invalid WAV header in %s", path);
			::fclose(f);
			return false;
		}

		if (::strncmp(riff_id, "RIFF", 4) != 0 || ::strncmp(wave_id, "WAVE", 4) != 0)
		{
			ROBOTICK_WARNING("Not a RIFF/WAVE file: %s", path);
			::fclose(f);
			return false;
		}

		bool have_fmt = false;
		bool have_data = false;
		uint16_t audio_format = 0;
		uint16_t bits_per_sample = 0;
		uint32_t data_size = 0;
		long data_pos = 0;

		while (!::feof(f) && (!have_fmt || !have_data))
		{
			char chunk_id[4];
			uint32_t chunk_size = 0;

			if (::fread(chunk_id, 1, 4, f) != 4 || !read_u32le(f, chunk_size))
			{
				ROBOTICK_WARNING("Unexpected EOF or corrupt chunk header in %s", path);
				::fclose(f);
				return false;
			}

			if (::strncmp(chunk_id, "fmt ", 4) == 0)
			{
				if (!read_u16le(f, audio_format) || !read_u16le(f, num_channels) || !read_u32le(f, sample_rate))
				{
					ROBOTICK_WARNING("Corrupt fmt chunk in %s", path);
					::fclose(f);
					return false;
				}

				uint32_t byte_rate;
				uint16_t block_align;
				if (!read_u32le(f, byte_rate) || !read_u16le(f, block_align) || !read_u16le(f, bits_per_sample))
				{
					ROBOTICK_WARNING("Corrupt fmt chunk in %s", path);
					::fclose(f);
					return false;
				}

				if (chunk_size > 16)
				{
					if (::fseek(f, static_cast<long>(chunk_size - 16), SEEK_CUR) != 0)
					{
						ROBOTICK_WARNING("Failed to skip extra fmt bytes in %s", path);
						::fclose(f);
						return false;
					}
				}

				have_fmt = true;
			}
			else if (::strncmp(chunk_id, "data", 4) == 0)
			{
				data_pos = ::ftell(f);
				data_size = chunk_size;
				have_data = true;
			}
			else
			{
				if (::fseek(f, static_cast<long>(chunk_size), SEEK_CUR) != 0)
				{
					ROBOTICK_WARNING("Failed to skip unknown chunk in %s", path);
					::fclose(f);
					return false;
				}
			}

			if (chunk_size & 1)
			{
				if (::fseek(f, 1, SEEK_CUR) != 0)
				{
					ROBOTICK_WARNING("Failed to skip padding byte in %s", path);
					::fclose(f);
					return false;
				}
			}
		}

		if (!have_fmt || !have_data || audio_format != 1 || bits_per_sample != 16 || (num_channels != 1 && num_channels != 2))
		{
			ROBOTICK_WARNING("Unsupported WAV format in %s", path);
			::fclose(f);
			return false;
		}

		::fseek(f, data_pos, SEEK_SET);
		size_t bytes_per_sample = bits_per_sample / 8;
		frame_count = data_size / (num_channels * bytes_per_sample);

		left_samples.initialize(frame_count);
		right_samples.initialize(frame_count);

		for (size_t i = 0; i < frame_count; ++i)
		{
			int16_t l = 0;
			int16_t r = 0;
			if (::fread(&l, sizeof(int16_t), 1, f) != 1)
			{
				ROBOTICK_WARNING("Unexpected EOF while reading samples in %s", path);
				::fclose(f);
				return false;
			}

			if (num_channels == 2)
			{
				if (::fread(&r, sizeof(int16_t), 1, f) != 1)
				{
					ROBOTICK_WARNING("Unexpected EOF while reading samples in %s", path);
					::fclose(f);
					return false;
				}
			}
			else
			{
				r = l;
			}

			left_samples[i] = static_cast<float>(l) / 32768.0f;
			right_samples[i] = static_cast<float>(r) / 32768.0f;
		}

		::fclose(f);
		return true;
	}

	float WavFile::get_duration_seconds() const
	{
		if (sample_rate == 0)
			return 0.0f;
		return static_cast<float>(frame_count) / static_cast<float>(sample_rate);
	}

	bool WavFileWriter::open(const char* path, uint32_t sr, uint16_t channels)
	{
		close();
		write_channels = channels;
		sample_rate = sr;
		fp = ::fopen(path, "wb+ ");
		if (!fp)
		{
			ROBOTICK_WARNING("WavFileWriter: failed to open %s", path);
			return false;
		}

		write_header_placeholder(sr, channels);
		return true;
	}

	void WavFileWriter::write_header_placeholder(uint32_t sr, uint16_t ch)
	{
		const uint32_t fmt_size = 16;
		const uint16_t format_tag = 1;
		const uint32_t byte_rate = sr * ch * sizeof(int16_t);
		const uint16_t block_align = ch * sizeof(int16_t);
		const uint16_t bits_per_sample = 16;
		const uint32_t riff_size_placeholder = 0;
		const uint32_t data_size_placeholder = 0;

		::fwrite("RIFF", 1, 4, fp);
		::fwrite(&riff_size_placeholder, 4, 1, fp);
		::fwrite("WAVE", 1, 4, fp);

		::fwrite("fmt ", 1, 4, fp);
		::fwrite(&fmt_size, 4, 1, fp);
		::fwrite(&format_tag, 2, 1, fp);
		::fwrite(&ch, 2, 1, fp);
		::fwrite(&sr, 4, 1, fp);
		::fwrite(&byte_rate, 4, 1, fp);
		::fwrite(&block_align, 2, 1, fp);
		::fwrite(&bits_per_sample, 2, 1, fp);

		::fwrite("data", 1, 4, fp);
		::fwrite(&data_size_placeholder, 4, 1, fp);
	}

	void WavFileWriter::append_mono(const float* samples, size_t count)
	{
		if (!fp || write_channels == 0 || !samples)
			return;

		for (size_t i = 0; i < count; ++i)
		{
			float v = samples[i];
			v = robotick::clamp(v, -1.0f, 1.0f);
			int16_t s16 = static_cast<int16_t>(::roundf(v * 32767.0f));
			::fwrite(&s16, sizeof(int16_t), 1, fp);
		}
		data_bytes_written += static_cast<uint32_t>(count * sizeof(int16_t));
	}

	void WavFileWriter::append_stereo(const float* left, const float* right, size_t count)
	{
		if (!fp || write_channels < 2)
		{
			append_mono(left, count);
			return;
		}

		for (size_t i = 0; i < count; ++i)
		{
			float l = left ? left[i] : 0.0f;
			float r = right ? right[i] : 0.0f;
			l = robotick::clamp(l, -1.0f, 1.0f);
			r = robotick::clamp(r, -1.0f, 1.0f);
			int16_t sL = static_cast<int16_t>(::roundf(l * 32767.0f));
			int16_t sR = static_cast<int16_t>(::roundf(r * 32767.0f));
			::fwrite(&sL, sizeof(int16_t), 1, fp);
			::fwrite(&sR, sizeof(int16_t), 1, fp);
		}
		data_bytes_written += static_cast<uint32_t>(count * sizeof(int16_t) * 2);
	}

	void WavFileWriter::close()
	{
		if (!fp)
			return;
		patch_header();
		::fclose(fp);
		fp = nullptr;
		write_channels = 0;
		data_bytes_written = 0;
	}

	void WavFileWriter::patch_header()
	{
		if (!fp)
			return;

		const uint32_t riff_size = data_bytes_written + 36;
		::fflush(fp);
		::fseek(fp, 4, SEEK_SET);
		::fwrite(&riff_size, 4, 1, fp);
		::fseek(fp, 40, SEEK_SET);
		::fwrite(&data_bytes_written, 4, 1, fp);
		::fflush(fp);
		::fseek(fp, 0, SEEK_END);
	}

} // namespace robotick
