
#include "robotick/systems/auditory/ProsodyState.h"

#include "robotick/api.h"

#include <cstdlib>
#include <cstring>

namespace robotick
{
	// ProsodyState Registration: ===========================

	ROBOTICK_REGISTER_STRUCT_BEGIN(ProsodyState)

	// ===== Core =====
	ROBOTICK_STRUCT_FIELD(ProsodyState, float, rms)
	ROBOTICK_STRUCT_FIELD(ProsodyState, float, zcr)
	ROBOTICK_STRUCT_FIELD(ProsodyState, float, pitch_hz)
	ROBOTICK_STRUCT_FIELD(ProsodyState, float, pitch_slope_hz_per_s)
	ROBOTICK_STRUCT_FIELD(ProsodyState, bool, voiced)
	ROBOTICK_STRUCT_FIELD(ProsodyState, float, voiced_confidence)

	// ===== Spectral summary =====
	ROBOTICK_STRUCT_FIELD(ProsodyState, float, spectral_energy_rms)
	ROBOTICK_STRUCT_FIELD(ProsodyState, float, spectral_energy_ratio)
	ROBOTICK_STRUCT_FIELD(ProsodyState, float, spectral_centroid_hz)
	ROBOTICK_STRUCT_FIELD(ProsodyState, float, spectral_bandwidth_hz)
	ROBOTICK_STRUCT_FIELD(ProsodyState, float, spectral_flatness)
	ROBOTICK_STRUCT_FIELD(ProsodyState, float, spectral_rolloff_hz)
	ROBOTICK_STRUCT_FIELD(ProsodyState, float, spectral_slope)

	// ===== Harmonicity / perturbation =====
	ROBOTICK_STRUCT_FIELD(ProsodyState, float, harmonicity_hnr_db)
	ROBOTICK_STRUCT_FIELD(ProsodyState, float, jitter)
	ROBOTICK_STRUCT_FIELD(ProsodyState, float, shimmer)

	// ===== Formants =====
	ROBOTICK_STRUCT_FIELD(ProsodyState, float, formant_f1_hz)
	ROBOTICK_STRUCT_FIELD(ProsodyState, float, formant_f2_hz)
	ROBOTICK_STRUCT_FIELD(ProsodyState, float, formant_f3_hz)

	// ===== Partials from analyser =====
	ROBOTICK_STRUCT_FIELD(ProsodyState, int, partial_count)
	ROBOTICK_STRUCT_FIELD(ProsodyState, ProsodyPartials, partial_gain)
	ROBOTICK_STRUCT_FIELD(ProsodyState, ProsodyPartials, partial_freq_hz)
	ROBOTICK_STRUCT_FIELD(ProsodyState, bool, partial_freq_valid)

	// ===== Speaking rate =====
	ROBOTICK_STRUCT_FIELD(ProsodyState, float, speaking_rate_sps)

	ROBOTICK_REGISTER_STRUCT_END(ProsodyState)

	// Partials Vector Registration: ===========================

	static bool partials_vector_to_string(const void* data, char* out_buffer, size_t buffer_size)
	{
		const ProsodyPartials* vec = static_cast<const ProsodyPartials*>(data);
		if (!vec || !out_buffer || buffer_size < 8)
			return false;

		size_t used = 0;
		int written = snprintf(out_buffer, buffer_size, "[");
		if (written < 0 || (size_t)written >= buffer_size)
			return false;
		used += (size_t)written;

		for (size_t i = 0; i < vec->size(); ++i)
		{
			const float value = (*vec)[i];
			written = snprintf(out_buffer + used, buffer_size - used, i == 0 ? "%.3f" : ", %.3f", value);
			if (written < 0 || used + (size_t)written >= buffer_size)
				break;
			used += (size_t)written;
		}

		if (used + 2 >= buffer_size)
			return false;

		strncpy(out_buffer + used, "]", buffer_size - used);
		return true;
	}

	static bool partials_vector_from_string(const char* str, void* data)
	{
		if (!str || !data)
			return false;

		ProsodyPartials* vec = static_cast<ProsodyPartials*>(data);
		vec->clear();

		// Copy to mutable buffer
		char temp[512];
		strncpy(temp, str, sizeof(temp));
		temp[sizeof(temp) - 1] = '\0';

		// Trim surrounding brackets if present
		char* start = temp;
		if (*start == '[')
			++start;
		char* end = strchr(start, ']');
		if (end)
			*end = '\0';

		// Parse comma-separated floats
		char* saveptr = nullptr;
		char* token = strtok_r(start, ",", &saveptr);
		while (token && vec->size() < vec->capacity())
		{
			float f = strtof(token, nullptr);
			vec->add(f);
			token = strtok_r(nullptr, ",", &saveptr);
		}

		return true;
	}

	ROBOTICK_REGISTER_PRIMITIVE(ProsodyPartials, partials_vector_to_string, partials_vector_from_string);

} // namespace robotick