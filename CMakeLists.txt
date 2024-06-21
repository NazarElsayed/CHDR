cmake_minimum_required(VERSION ${CMAKE_VERSION})
project(Pathfinding)

set(CMAKE_CXX_STANDARD 17)

# DETECT ARCHITECTURE FOR SIMD OPTIMISATION
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86|AMD64)$")
    set(ARCH_SSE2_FLAGS "-msse2")
    set(ARCH_AVX_FLAGS "-mavx")
    set(ARCH_AVX2_FLAGS "-mavx2")
    set(ARCH_AVX512_FLAGS "-mavx512f -mavx512dq -mavx512bw -mavx512vl")
endif()

# APPEND ARCHITECTURE-SPECIFIC FLAGS FOR SIMD OPTIMISATION
if(ARCH_SSE2_FLAGS)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_RELEASE} ${ARCH_SSE2_FLAGS}")
endif()
if(ARCH_AVX_FLAGS)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_RELEASE} ${ARCH_AVX_FLAGS}")
endif()
if(ARCH_AVX2_FLAGS)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_RELEASE} ${ARCH_AVX2_FLAGS}")
endif()
if(ARCH_AVX512_FLAGS)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_RELEASE} ${ARCH_AVX512_FLAGS}")
endif()

# DIFFERENT BUILD MODES:
set(CMAKE_CXX_FLAGS_DEBUG "-fsanitize=address,leak,undefined -Wall -Wextra -fno-omit-frame-pointer -g -O0")

set(CMAKE_CXX_FLAGS_NOSANITIZERS "-Wall -Wextra -fno-omit-frame-pointer -g -O0")

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=native -funroll-loops -DNDEBUG")

## CORE:

# ADD PATHFINDING:
add_library(Pathfinding STATIC
        src/core/AStar/AStar.hpp
        src/core/foo.cpp
)

target_precompile_headers(Pathfinding PRIVATE src/core/stdafx.hpp)

# LINK LIBS:
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    target_link_options(Pathfinding PRIVATE -flto=auto -Wl,--strip-all -Wl,--gc-sections -Wl,--as-needed -Wl,--no-undefined)
else()
    target_link_options(Pathfinding PRIVATE -flto=auto -Wl,--gc-sections -Wl,--as-needed -Wl,--no-undefined)
endif()

## TEST:

# ADD EXECUTABLE:
add_executable(Test
        src/test/main.cpp
)

if(CMAKE_BUILD_TYPE STREQUAL "Release")
    target_link_options(Test PRIVATE  -flto=auto -Wl,--strip-all -Wl,--gc-sections -Wl,--as-needed -Wl,--no-undefined)
else()
    target_link_options(Test PRIVATE -flto=auto -Wl,--gc-sections -Wl,--as-needed -Wl,--no-undefined)
endif()

target_link_libraries(Test PRIVATE Pathfinding)