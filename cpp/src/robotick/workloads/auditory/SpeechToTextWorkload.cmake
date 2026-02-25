# ===================================================================
# Robotick Speech-to-Text Integration (Whisper.cpp backend)
# ===================================================================
# Provides target: whisper
# Optimized with AVX2/FMA/F16C/BMI2 + OpenMP + fast-math.
# Can be safely included from Launcher, Engine, or Workloads.
# ===================================================================

cmake_minimum_required(VERSION 3.16)

include(FetchContent)
include(CheckCXXCompilerFlag)

message(STATUS "[SpeechToText] Configuring Whisper.cpp integration...")

# --------------------------------------------------
# Disable unneeded Whisper.cpp components
# --------------------------------------------------
set(WHISPER_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(WHISPER_BUILD_TESTS    OFF CACHE BOOL "" FORCE)
set(WHISPER_BUILD_SERVER   OFF CACHE BOOL "" FORCE)
set(WHISPER_SDL2           OFF CACHE BOOL "" FORCE)
set(WHISPER_FFMPEG         OFF CACHE BOOL "" FORCE)
set(WHISPER_COREML         OFF CACHE BOOL "" FORCE)
set(WHISPER_OPENVINO       OFF CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS      OFF CACHE BOOL "" FORCE)

# --------------------------------------------------
# Fetch Whisper.cpp from upstream
# --------------------------------------------------
FetchContent_Declare(
    whispercpp
    GIT_REPOSITORY https://github.com/ggerganov/whisper.cpp.git
    GIT_TAG        v1.8.2
)
FetchContent_MakeAvailable(whispercpp)

# --------------------------------------------------
# CPU SIMD feature detection
# --------------------------------------------------
message(STATUS "[SpeechToText] Checking SIMD feature support...")

check_cxx_compiler_flag("-mavx2" COMPILER_SUPPORTS_AVX2)
check_cxx_compiler_flag("-mfma"  COMPILER_SUPPORTS_FMA)
check_cxx_compiler_flag("-mf16c" COMPILER_SUPPORTS_F16C)
check_cxx_compiler_flag("-mbmi2" COMPILER_SUPPORTS_BMI2)

set(SIMD_FLAGS "")
set(SIMD_DEFS "")

if (COMPILER_SUPPORTS_AVX2)
    list(APPEND SIMD_FLAGS "-mavx2")
    list(APPEND SIMD_DEFS "GGML_USE_AVX2")
endif()
if (COMPILER_SUPPORTS_FMA)
    list(APPEND SIMD_FLAGS "-mfma")
endif()
if (COMPILER_SUPPORTS_F16C)
    list(APPEND SIMD_FLAGS "-mf16c")
    list(APPEND SIMD_DEFS "GGML_USE_F16C")
endif()
if (COMPILER_SUPPORTS_BMI2)
    list(APPEND SIMD_FLAGS "-mbmi2")
    list(APPEND SIMD_DEFS "GGML_USE_BMI2")
endif()

if (SIMD_FLAGS)
    message(STATUS "[SpeechToText] Enabling SIMD flags: ${SIMD_FLAGS}")
else()
    message(WARNING "[SpeechToText] No SIMD instruction set flags supported")
endif()

# --------------------------------------------------
# OpenMP multithreading
# --------------------------------------------------
find_package(OpenMP)
if (OpenMP_CXX_FOUND)
    message(STATUS "[SpeechToText] OpenMP found — enabling multithreaded decoding")
    add_compile_definitions(WHISPER_USE_OPENMP)
else()
    message(WARNING "[SpeechToText] OpenMP not found — single-threaded fallback")
endif()

# --------------------------------------------------
# Apply high-optimization build settings
# --------------------------------------------------
foreach(tgt whisper ggml ggml-cpu)
    if (TARGET ${tgt})
		if (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
			target_compile_options(${tgt} PRIVATE /O2 /DNDEBUG)
		else()
			target_compile_options(${tgt} PRIVATE
				${SIMD_FLAGS}
				-O3 -g0
				-ffast-math -funsafe-math-optimizations
				-fno-finite-math-only
				-DNDEBUG
			)
		endif()
        target_compile_definitions(${tgt} PRIVATE
            WHISPER_VERSION=\"1.8.2\"
            WHISPER_USE_FLASH_ATTN
            ${SIMD_DEFS}
        )
        if (OpenMP_CXX_FOUND)
            target_link_libraries(${tgt} PRIVATE OpenMP::OpenMP_CXX)
        endif()
    endif()
endforeach()

message(STATUS "[SpeechToText] Whisper.cpp integration ready (optimized build)")
