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

# DIFFERENT BUILD MODES:
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -fsanitize=address,leak,undefined -Wall -Wextra -fno-omit-frame-pointer -fno-rtti -g")

set(CMAKE_CXX_FLAGS_NOSANITIZERS "-O0 -Wall -Wextra -fno-omit-frame-pointer -fno-rtti -g")

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Ofast -flto -march=native -mtune=native -funroll-loops -ftree-vectorize -ffast-math -fstrict-aliasing -fomit-frame-pointer -fno-rtti -DNDEBUG")

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
        src/core/solvers/base/managed_node.hpp
        src/core/solvers/base/solver.hpp
        src/core/solvers/bfs.hpp
        src/core/solvers/bstar.hpp
        src/core/solvers/dfs.hpp
        src/core/solvers/dijkstra.hpp
        src/core/solvers/eidastar.hpp
        src/core/solvers/eidbstar.hpp
        src/core/solvers/eiddfs.hpp
        src/core/solvers/esmgstar.hpp
        src/core/solvers/floodfill.hpp
        src/core/solvers/fstar.hpp
        src/core/solvers/gbfs.hpp
        src/core/solvers/gdfs.hpp
        src/core/solvers/idastar.hpp
        src/core/solvers/idbstar.hpp
        src/core/solvers/iddfs.hpp
        src/core/solvers/jps.hpp
        src/core/types/append_only_allocator.hpp
        src/core/types/circular_queue.hpp
        src/core/types/coord.hpp
        src/core/types/dynamic_pool_allocator.hpp
        src/core/types/existence_set.hpp
        src/core/types/heap.hpp
        src/core/types/queue.hpp
        src/core/types/stack.hpp
        src/core/types/stack_allocator.hpp
        src/core/utils/heuristics.hpp
        src/core/utils/intrinsics.hpp
        src/core/utils/utils.hpp
)

target_link_libraries(CHDR INTERFACE)

target_precompile_headers(CHDR INTERFACE src/core/stdafx.hpp)

## LINK OPTIONS:
if (CMAKE_BUILD_TYPE STREQUAL "Release")
    target_link_options(CHDR INTERFACE -flto=auto -Wl,--strip-all -Wl,--gc-sections -Wl,--as-needed -Wl,--no-undefined)
else ()
    target_link_options(CHDR INTERFACE -flto=auto -Wl,--gc-sections -Wl,--as-needed -Wl,--no-undefined)
endif ()

## TEST:

# ADD EXECUTABLE:
add_executable(Test
        src/test/main.cpp

        src/test/scripts/core/application.hpp
        src/test/scripts/core/display.hpp
        src/test/scripts/generator/graph.hpp
        src/test/scripts/generator/grid.hpp
        src/test/scripts/generator/utils/backtracking.hpp
        src/test/scripts/generator/utils/lcg.hpp
        src/test/scripts/tests/astar.hpp
)

target_include_directories(Test PRIVATE
        src/core
)

target_precompile_headers(CHDR INTERFACE src/core/stdafx.hpp)

## LINK OPTIONS:
if (UNIX)
    if (CMAKE_BUILD_TYPE STREQUAL "Release")
        target_link_options(Test PRIVATE -flto=auto -Wl,--strip-all -Wl,--gc-sections -Wl,--as-needed -Wl,--no-undefined)
    else ()
        target_link_options(Test PRIVATE -flto=auto -Wl,--gc-sections -Wl,--as-needed -Wl,--no-undefined)
    endif ()
endif (UNIX)

target_link_libraries(Test INTERFACE
        LOUIERIKSSON_DEBUG_HPP
        CHDR
)