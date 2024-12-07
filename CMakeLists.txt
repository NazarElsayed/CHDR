# Computational Helper for Direction and Routing (CHDR)
# Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
#
# Licensed under CC BY-NC-ND 4.0
# https://creativecommons.org/licenses/by-nc-nd/4.0/

cmake_minimum_required(VERSION ${CMAKE_VERSION})

project(CHDR VERSION 0.1.0 LANGUAGES CXX)

# Add an optional tag to the end of the version.
set(CHDR_VERSION_METADATA "-alpha")
if (CHDR_VERSION_METADATA)
    set(CHDR_VERSION "${CHDR_VERSION_MAJOR}.${CHDR_VERSION_MINOR}.${CHDR_VERSION_PATCH}${CHDR_VERSION_METADATA}")
endif ()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

configure_file(CHDR_Version.hpp.in CHDR_Version.hpp)

# DETECT ARCHITECTURE FOR SIMD OPTIMISATION:

if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(i[0-9]86|x86_64|AMD64)$") # x86 Processors
    set(SIMD_FLAGS
        -mmmx       # <- MMX Flags
        -m3dnow     # <- 3DNOW
        -msse       # <- SSE Flags
        -msse2      # <- SSE2 Flags
        -msse3      # <- SSE3 Flags
        -mssse3     # <- Supplemental SSE3 Flags
        -msse4.1    # <- SSE4.1 Flags
        -msse4.2    # <- SSE4.2 Flags
        -mavx       # <- AVX Flags
        -mavx2      # <- AVX2 Flags
        #-mavx512f -mavx512dq -mavx512bw -mavx512vl  # <- AVX-512 Flags
    )

elseif(${CMAKE_SYSTEM_PROCESSOR} MATCHES "^ppc64le") #PowerPC Processors
    set(SIMD_FLAGS
        -maltivec   # <- AltiVec/VMX Flags
        -mvsx       # <- VSX Flags
    )

elseif(${CMAKE_SYSTEM_PROCESSOR} MATCHES "^arm") #ARM Processors
    set(SIMD_FLAGS
        -mfpu=neon  # <- NEON Flags
        -march=armv8-a+crc+crypto
    )

elseif(${CMAKE_SYSTEM_PROCESSOR} MATCHES "^aarch64") #ARM64 Processors
    set(SIMD_FLAGS
        -march=armv8-a  # <- needed to have NEON support
    )

elseif(${CMAKE_SYSTEM_PROCESSOR} MATCHES "^sparc") #SPARC Processors
    set(SIMD_FLAGS
        -mv8
        -mvis   # <- Visual Instruction Set Flags
        -mvis2
    )

elseif (${CMAKE_SYSTEM_PROCESSOR} MATCHES "^mips") #MIPS Processors
    set(SIMD_FLAGS
            -mdspr2 # <- MIPS DSPr2 Flag
    )

elseif(${CMAKE_SYSTEM_PROCESSOR} MATCHES "^ia64") #Itanium Processors
    set(SIMD_FLAGS
        -march=itanium-2
    )

elseif(${CMAKE_SYSTEM_PROCESSOR} MATCHES "^(s390|s390x)$") # IBM S/390 and z Series Processors
    set(SIMD_FLAGS
        -march=z13
    )

elseif(${CMAKE_SYSTEM_PROCESSOR} MATCHES "^riscv") #RISC-V Processors
    set(SIMD_FLAGS
        -march=rv64imafdc -mabi=lp64d
    )

elseif(${CMAKE_SYSTEM_PROCESSOR} MATCHES "^alpha") #DEC Alpha Processors
    set(SIMD_FLAGS
        -mieee
    )

elseif(${CMAKE_SYSTEM_PROCESSOR} MATCHES "^hppa") #HP PA-RISC Processors
    set(SIMD_FLAGS
        -mschedule=8000
    )

elseif(${CMAKE_SYSTEM_PROCESSOR} MATCHES "^(68000|68040|68060)$") #Motorola 68k Processors
    set(SIMD_FLAGS
        -m68060
    )

elseif(${CMAKE_SYSTEM_PROCESSOR} MATCHES "^(sh4|sh4a)$") #SuperH Processors
    set(SIMD_FLAGS
        -m4a
    )

else()
    set(SIMD_FLAGS "")
endif()

add_compile_options(${SIMD_FLAGS})

# DIFFERENT BUILD MODES:
set(CMAKE_CXX_FLAGS_DEBUG "-fsanitize=address,leak,undefined -Wall -Wextra -fno-omit-frame-pointer -g -O0")

set(CMAKE_CXX_FLAGS_NOSANITIZERS "-Wall -Wextra -fno-omit-frame-pointer -g -O0")

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=native -mtune=native -funroll-loops -ftree-vectorize -ffast-math -DNDEBUG")

include_directories(
    ${CHDR_BINARY_DIR}
    contrib/louieriksson
)

## LOUIERIKSSON::DEBUG:
add_library(LOUIERIKSSON_DEBUG_HPP INTERFACE
    contrib/louieriksson/debug.hpp
)

## CORE:

# ADD CHDR:
add_library(CHDR INTERFACE
    src/core/chdr.hpp
    src/core/mazes/base/igraph.hpp
    src/core/mazes/base/imaze.hpp
    src/core/mazes/graph.hpp
    src/core/mazes/grid.hpp
    src/core/mazes/nodes/base/inode.hpp
    src/core/mazes/nodes/id_node.hpp
    src/core/mazes/nodes/weighted_node.hpp
    src/core/solvers/astar.hpp
    src/core/solvers/base/bnode.hpp
    src/core/solvers/base/bsolver.hpp
    src/core/solvers/base/managed_node.hpp
    src/core/solvers/bfs.hpp
    src/core/solvers/bstar.hpp
    src/core/solvers/dfs.hpp
    src/core/solvers/dijkstra.hpp
    src/core/solvers/esmgstar.hpp
    src/core/solvers/floodfill.hpp
    src/core/solvers/gbfs.hpp
    src/core/solvers/gdfs.hpp
    src/core/solvers/gstar.hpp
    src/core/solvers/jps.hpp
    src/core/types/circular_queue.hpp
    src/core/types/coord.hpp
    src/core/types/existence_set.hpp
    src/core/types/heap.hpp
    src/core/types/linear_priority_queue.hpp
    src/core/types/queue.hpp
    src/core/types/stable_forward_buf.hpp
    src/core/types/stack.hpp
    src/core/types/stack_allocator.hpp
    src/core/utils/heuristics.hpp
    src/core/utils/intrinsics.hpp
    src/core/utils/simd_extensions.hpp
    src/core/utils/utils.hpp
)

target_link_libraries(CHDR INTERFACE)

target_precompile_headers(CHDR INTERFACE src/core/stdafx.hpp)

# LINK OPTIONS:
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    target_link_options(CHDR INTERFACE -flto=auto -Wl,--strip-all -Wl,--gc-sections -Wl,--as-needed -Wl,--no-undefined)
else()
    target_link_options(CHDR INTERFACE -flto=auto -Wl,--gc-sections -Wl,--as-needed -Wl,--no-undefined)
endif()

## TEST:

# ADD EXECUTABLE:
add_executable(Test
    src/test/main.cpp

    src/test/scripts/core/application.hpp
    src/test/scripts/core/display.hpp
    src/test/scripts/generator/graph.hpp
    src/test/scripts/generator/grid.hpp
    src/test/scripts/generator/utils/backtracking.hpp
    src/test/scripts/tests/astar.hpp
)

target_include_directories(Test PRIVATE
    src/core
)

target_precompile_headers(CHDR INTERFACE src/core/stdafx.hpp)

# LINK OPTIONS:
if(UNIX)
    if(CMAKE_BUILD_TYPE STREQUAL "Release")
        target_link_options(Test PRIVATE -flto=auto -Wl,--strip-all -Wl,--gc-sections -Wl,--as-needed -Wl,--no-undefined)
    else()
        target_link_options(Test PRIVATE -flto=auto -Wl,--gc-sections -Wl,--as-needed -Wl,--no-undefined)
    endif()
endif(UNIX)

target_link_libraries(Test INTERFACE
    LOUIERIKSSON_DEBUG_HPP
    CHDR
)