cmake_minimum_required(VERSION ${CMAKE_VERSION})
project(CHDR LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

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