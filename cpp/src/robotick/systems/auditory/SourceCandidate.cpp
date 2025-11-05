
#include "robotick/systems/auditory/SourceCandidate.h"

#include "robotick/api.h"

#include <cstdlib>
#include <cstring>

namespace robotick
{
	// ProsodyState Registration: ===========================

	ROBOTICK_REGISTER_STRUCT_BEGIN(SourceCandidate)
	ROBOTICK_STRUCT_FIELD(SourceCandidate, float, h1_f0_hz)
	ROBOTICK_STRUCT_FIELD(SourceCandidate, HarmonicAmplitudes, harmonic_amplitudes)
	ROBOTICK_REGISTER_STRUCT_END(SourceCandidate)

	// Partials Vector Registration: ===========================

	static bool candidates_vector_to_string(const void* data, char* out_buffer, size_t buffer_size)
	{
		const SourceCandidates8* buf = static_cast<const SourceCandidates8*>(data);
		if (!buf || !out_buffer || buffer_size < 32)
			return false;

		// Format: <SourceCandidates8(size)>
		int written = snprintf(out_buffer, buffer_size, "<SourceCandidates8(%zu)>", buf->size());
		return written > 0 && static_cast<size_t>(written) < buffer_size;
	}

	static bool candidates_vector_from_string(const char*, void*)
	{
		// Read-only string representation, parsing not supported
		return false;
	}

	ROBOTICK_REGISTER_PRIMITIVE(SourceCandidates8, candidates_vector_to_string, candidates_vector_from_string);

} // namespace robotick