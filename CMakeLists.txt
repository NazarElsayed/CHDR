cmake_minimum_required(VERSION ${CMAKE_VERSION})
project(CHDR LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
# DETECT ARCHITECTURE FOR SIMD OPTIMISATION
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86|AMD64)$")
    set(ARCH_MMX_FLAGS "-mmmx")  # <- MMX Flags
    set(ARCH_SSE_FLAGS "-msse")  # <- SSE Flags
    set(ARCH_SSE2_FLAGS "-msse2") # <- SSE2 Flags
    set(ARCH_SSE3_FLAGS "-msse3")  # <- SSE3 Flags
    set(ARCH_SSSE3_FLAGS "-mssse3")  # <- Supplimental SSE3 Flags
    set(ARCH_SSE4_1_FLAGS "-msse4.1")  # <- SSE4.1 Flags
    set(ARCH_SSE4_2_FLAGS "-msse4.2")  # <- SSE4.2 Flags
    set(ARCH_AVX_FLAGS "-mavx") # <- AVX Flags
    set(ARCH_AVX2_FLAGS "-mavx2") # <- AVX2 Flags
    set(ARCH_AVX512_FLAGS "-mavx512f -mavx512dq -mavx512bw -mavx512vl") # <- AVX-512 Flags
endif()

# APPEND ARCHITECTURE-SPECIFIC FLAGS FOR SIMD OPTIMISATION
if(ARCH_MMX_FLAGS)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_RELEASE} ${ARCH_MMX_FLAGS}")
endif()
if(ARCH_SSE_FLAGS)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_RELEASE} ${ARCH_SSE_FLAGS}")
endif()
if(ARCH_SSE2_FLAGS)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_RELEASE} ${ARCH_SSE2_FLAGS}")
endif()
if(ARCH_SSE3_FLAGS)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_RELEASE} ${ARCH_SSE3_FLAGS}")
endif()
if(ARCH_SSSE3_FLAGS)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_RELEASE} ${ARCH_SSSE3_FLAGS}")
endif()
if(ARCH_SSE4_1_FLAGS)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_RELEASE} ${ARCH_SSE4_1_FLAGS}")
endif()
if(ARCH_SSE4_2_FLAGS)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_RELEASE} ${ARCH_SSE4_2_FLAGS}")
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

        src/core/types/Node.hpp

        src/core/solvers/AStar.hpp
        src/core/solvers/base/ISolver.hpp
        src/core/solvers/Dijkstra.hpp
        src/core/solvers/JPS.hpp

        src/core/utils/Utils.hpp
        src/core/types/Heap.hpp
        src/core/types/RelationalNode.hpp
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