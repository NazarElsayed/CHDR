cmake_minimum_required(VERSION ${CMAKE_VERSION})
project(CHDR LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

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

elseif(${CMAKE_SYSTEM_PROCESSOR} MATCHES "^mips") #MIPS Processors
    set(SIMD_FLAGS
        -mdspr2 # <- MIPS DSPr2 Flag
        -mmmx   # <- MMX Flags
        -msse   # <- SSE Flags
        -msse2  # <- SSE2 Flags
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

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=native -mtune=native -funroll-loops -DNDEBUG")

include_directories(
        contrib/LouiEriksson
)

## LOUIERIKSSON::DEBUG:

add_library(LOUIERIKSSON_DEBUG_HPP INTERFACE
    contrib/LouiEriksson/Debug.hpp
)

## CORE:

# ADD CHDR:
add_library(CHDR INTERFACE
        src/core/chdr.hpp

        src/core/mazes/base/IMaze.hpp
        src/core/mazes/Graph.hpp
        src/core/mazes/Grid.hpp
        src/core/mazes/Tree.hpp

        src/core/types/DenseExistenceSet.hpp
        src/core/types/Heap.hpp
        src/core/types/Node.hpp
        src/core/types/RelationalNode.hpp

        src/core/solvers/AStar.hpp
        src/core/solvers/base/ISolver.hpp
        src/core/solvers/Dijkstra.hpp
        src/core/solvers/JPS.hpp

        src/core/utils/Utils.hpp
        src/core/types/HeavyNode.hpp
        src/core/types/NodeData.hpp
        src/core/mazes/MutableGrid.hpp
)

target_link_libraries(CHDR INTERFACE
        LOUIERIKSSON_DEBUG_HPP
)

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

        src/test/scripts/core/Application.hpp
        src/test/scripts/generator/Grid.hpp
        src/test/scripts/generator/utils/Backtracking.hpp
        src/test/scripts/tests/AStar.hpp
        src/test/scripts/core/Display.hpp
)

target_include_directories(Test PRIVATE
        src/core
)

target_precompile_headers(CHDR INTERFACE src/core/stdafx.hpp)

# LINK OPTIONS:
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    target_link_options(Test PRIVATE  -flto=auto -Wl,--strip-all -Wl,--gc-sections -Wl,--as-needed -Wl,--no-undefined)
else()
    target_link_options(Test PRIVATE -flto=auto -Wl,--gc-sections -Wl,--as-needed -Wl,--no-undefined)
endif()

target_link_libraries(Test INTERFACE
        LOUIERIKSSON_DEBUG_HPP
        CHDR
)