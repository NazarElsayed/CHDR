# Computational Helper for Direction and Routing (CHDR)
# Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
#
# Licensed under CC BY-NC-ND 4.0
# https://creativecommons.org/licenses/by-nc-nd/4.0/

cmake_minimum_required(VERSION ${CMAKE_VERSION})

project(CHDR VERSION 0.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

message(NOTICE "NOTICE: Checking for local Git repository:")
execute_process(
        COMMAND git rev-parse --is-inside-work-tree
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        RESULT_VARIABLE IS_GIT_REPO
        OUTPUT_QUIET
)

if (IS_GIT_REPO EQUAL 0)
    message(STATUS "STATUS: SUCCESS detecting local Git repository.")

    # Extract version from tags.
    execute_process(
        COMMAND git describe --tags
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        OUTPUT_VARIABLE GIT_VERSION
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )

    # Get the current branch.
    execute_process(
        COMMAND git rev-parse --abbrev-ref HEAD
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        OUTPUT_VARIABLE GIT_BRANCH
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )

    # Get the current commit hash.
    execute_process(
        COMMAND git rev-parse --short HEAD
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        OUTPUT_VARIABLE GIT_COMMIT_HASH
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
else()
    message(STATUS "STATUS: FAILED detecting local Git repository.")
endif()

message(NOTICE "NOTICE: Detecting CHDR version:")

# Use git version or fallback to a default version.
if (GIT_VERSION)
    string(REGEX REPLACE "^v([0-9]+\\.[0-9]+\\.[0-9]+).*" "\\1" GIT_VERSION "${GIT_VERSION}")

    # Extract MAJOR, MINOR, and PATCH versions directly from GIT_VERSION
    if (GIT_VERSION)
        string(REGEX MATCH "^([0-9]+)\\.([0-9]+)\\.([0-9]+)" _MATCHES "${GIT_VERSION}")
        set(CHDR_VERSION_MAJOR ${CMAKE_MATCH_1})
        set(CHDR_VERSION_MINOR ${CMAKE_MATCH_2})
        set(CHDR_VERSION_PATCH ${CMAKE_MATCH_3})
    endif ()
    message(STATUS "STATUS: SUCCESS determining CHDR version from git tags.")
else()
    message(WARNING "WARNING: FAILED automatically determining CHDR version from git tags.")

    message(STATUS "STATUS: Attempting to read version from chdr.hpp...")
    file(READ "${CMAKE_SOURCE_DIR}/chdr.hpp" CHDR_HEADER_CONTENTS)

    if (CHDR_HEADER_CONTENTS)
        foreach (VARIABLE MAJOR MINOR PATCH)
            string(REGEX MATCH "#define[ \t]+CHDR_VERSION_${VARIABLE}[ \t]+\"?([^\n\"]*)\"?" _MATCH "${CHDR_HEADER_CONTENTS}")
            set("CHDR_VERSION_${VARIABLE}" "${CMAKE_MATCH_1}")
        endforeach ()
    else ()
        message(FATAL_ERROR "FATAL_ERROR: Could not deduce version from chdr.hpp. Project must be versioned.")
    endif ()
endif()

if (NOT GIT_BRANCH OR GIT_BRANCH MATCHES "^(master|main)$")
    set(CHDR_VERSION_TWEAK "dev")
    set(CHDR_VERSION_METADATA ${GIT_COMMIT_HASH})
else ()
    string(REGEX REPLACE "[^a-zA-Z0-9\\-]" "_" GIT_BRANCH_SANITIZED ${GIT_BRANCH})
    set(CHDR_VERSION_TWEAK ${GIT_BRANCH_SANITIZED})
endif ()

if (CHDR_VERSION_TWEAK)
    set(CHDR_VERSION "${CHDR_VERSION_MAJOR}.${CHDR_VERSION_MINOR}.${CHDR_VERSION_PATCH}-${CHDR_VERSION_TWEAK}")
endif ()

if (CHDR_VERSION_METADATA)
    set(CHDR_VERSION "${CHDR_VERSION}+${CHDR_VERSION_METADATA}")
endif ()

message(STATUS "STATUS: CHDR_VERSION determined as \"${CHDR_VERSION}\"")

message(NOTICE "NOTICE: Generating chdr.hpp:")

# GENERATE "chdr.hpp":
file(GLOB_RECURSE HEADER_FILES "${CMAKE_SOURCE_DIR}/include/*.hpp")
set(INCLUDE_HEADERS "")
foreach (HEADER ${HEADER_FILES})
    file(RELATIVE_PATH RELATIVE_HEADER "${CMAKE_SOURCE_DIR}" "${HEADER}")
    set(INCLUDE_HEADERS "${INCLUDE_HEADERS}#include \"${RELATIVE_HEADER}\"\n")
endforeach ()

configure_file(chdr.hpp.in ${CMAKE_SOURCE_DIR}/chdr.hpp)

message(NOTICE "NOTICE: Building test application:")

set(WARNINGS_BASE "-Wall -Wextra -Wpedantic -Wconversion -Wshadow -Wnon-virtual-dtor -Wold-style-cast -Wcast-align -Woverloaded-virtual -Wnull-dereference -Wdouble-promotion -Wvla -Wformat=2 -Wmissing-declarations -Wsuggest-override -Wstrict-overflow=5 -Wuninitialized -Wswitch-enum -Wswitch-default -Wcast-qual -Wwrite-strings -Wpointer-arith -Wfloat-equal -Wc++20-extensions")

if (DEFINED SUPER_PEDANTIC)
    message(WARNING "WARNING: \"SUPER_PEDANTIC\" enabled. Build system may catch warnings as errors.")
    set(WARNINGS "${WARNINGS_BASE} -pedantic -Werror")
else ()
    set(WARNINGS "${WARNINGS_BASE}")
endif ()

# DIFFERENT BUILD MODES:
set(CMAKE_CXX_FLAGS_DEBUG        "${WARNINGS} -O0 -fsanitize=address,leak,undefined -fno-omit-frame-pointer -fno-rtti -g")
set(CMAKE_CXX_FLAGS_NOSANITIZERS "${WARNINGS} -O0                                   -fno-omit-frame-pointer -fno-rtti -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${WARNINGS} -O2 -flto -march=native -mtune=native -funroll-loops -ftree-vectorize -fstrict-aliasing -fomit-frame-pointer -fno-rtti -DNDEBUG")

# DEFINE LIBRARIES:
add_library(libdebug INTERFACE test/external/debug.hpp)
add_library(libchdr  INTERFACE                chdr.hpp)

# TEST:
add_executable(chdr test/main.cpp)

target_include_directories(chdr PRIVATE
        test/external/
        test/include/
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_precompile_headers(chdr INTERFACE stdafx.hpp)

## LINK OPTIONS:
if (UNIX)
    if (CMAKE_BUILD_TYPE STREQUAL "Release")
        target_link_options(chdr PRIVATE -flto=auto -Wl,--strip-all -Wl,--gc-sections -Wl,--as-needed -Wl,--no-undefined)
    else ()
        target_link_options(chdr PRIVATE -flto=auto -Wl,--gc-sections -Wl,--as-needed -Wl,--no-undefined)
    endif ()
endif (UNIX)

target_link_libraries(chdr INTERFACE
        libchdr
        libdebug
)