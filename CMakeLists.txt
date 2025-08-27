# Computational Helper for Direction and Routing (CHDR)
# Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
#
# Licensed under CC BY-NC-ND 4.0
# https://creativecommons.org/licenses/by-nc-nd/4.0/

cmake_minimum_required(VERSION ${CMAKE_VERSION})

project(CHDR VERSION 0.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
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
    set(CHDR_VERSION_FULL "${CHDR_VERSION}+${CHDR_VERSION_METADATA}")
else ()
    set(CHDR_VERSION_FULL "${CHDR_VERSION}")
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

message(NOTICE "Generating README.md")
configure_file(README.md.in ${CMAKE_SOURCE_DIR}/README.md)

message(NOTICE "NOTICE: Building test application:")

# EXTREME WARNINGS:
set(WARNINGS_BASE "-Wall -Wextra -Wpedantic -Wformat-security -Wconversion -Wsign-conversion -Wimplicit-fallthrough -Wshadow -Wnon-virtual-dtor -Wold-style-cast -Wcast-align -Woverloaded-virtual -Wnull-dereference -Wdouble-promotion -Wvla -Wformat=2 -Wmissing-declarations -Wsuggest-override -Wstrict-overflow=5 -Wuninitialized -Wswitch-enum -Wswitch-default -Wcast-qual -Wwrite-strings -Wpointer-arith -Wfloat-equal -Wtrampolines") # -Wbidi-chars=any") # -Wc++20-extensions

if (DEFINED SUPER_PEDANTIC)
    message(WARNING "WARNING: \"SUPER_PEDANTIC\" enabled. Build system may catch warnings as errors.")
    set(WARNINGS "${WARNINGS_BASE} -pedantic -Werror")
else ()
    set(WARNINGS "${WARNINGS_BASE}")
endif ()

# HIGHLIGHTING FOR COMPILER ERRORS:
if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fdiagnostics-color=always")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -freorder-blocks-and-partition")
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fcolor-diagnostics")
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /FC")
endif()

# DIFFERENT BUILD MODES:
set(CMAKE_CXX_FLAGS_DEBUG        "${WARNINGS} -O0 -fsanitize=address,leak,undefined -fno-inline -fno-strict-aliasing -fno-omit-frame-pointer -fno-rtti -fno-verbose-asm -flto -g")
set(CMAKE_CXX_FLAGS_NOSANITIZERS "${WARNINGS} -O3                                   -fno-inline -fno-strict-aliasing -fno-omit-frame-pointer -fno-rtti -fno-verbose-asm -flto -g -gdwarf-5")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${WARNINGS} -O2 -march=native -mtune=native -funroll-loops -ftree-vectorize -ffast-math -fstrict-aliasing -fno-strict-overflow -fdata-sections -ffunction-sections -fomit-frame-pointer -fno-rtti -DNDEBUG")

# DEFINE LIBRARIES:
add_library(libdebug INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/test/external/debug.hpp)
target_include_directories(libdebug INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/test/external)

add_library(libchdr INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/chdr.hpp)
target_include_directories(libchdr INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

# TEST:
add_executable(chdr test/main.cpp)

target_include_directories(chdr PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/test/include
)

target_link_libraries(chdr PRIVATE
        libchdr
        libdebug
)

target_precompile_headers(chdr INTERFACE stdafx.hpp)

## LINK OPTIONS:
if (UNIX)
    if (CMAKE_BUILD_TYPE STREQUAL "Release")
        target_link_options(chdr PRIVATE -Wl,--strip-all -Wl,--gc-sections -Wl,--as-needed -Wl,--no-undefined)
    else ()
        target_link_options(chdr PRIVATE -Wl,--gc-sections -Wl,--as-needed -Wl,--no-undefined)
    endif ()
else()
    target_link_options(chdr PRIVATE -static)
endif (UNIX)

message(NOTICE "NOTICE: Generating documentation with Doxygen:")
configure_file(Doxyfile.in ${CMAKE_SOURCE_DIR}/Doxyfile)
message(STATUS "Generating Doxyfile...")
execute_process(
    COMMAND doxygen ${CMAKE_SOURCE_DIR}/Doxyfile
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    RESULT_VARIABLE DOXYGEN_RESULT
)

message(STATUS "Cleaning up...")

if (DOXYGEN_RESULT EQUAL 0)

    # Clean up potential junk files left by Doxygen.
    if (EXISTS "${CMAKE_SOURCE_DIR}/docs/html")
        file(REMOVE_RECURSE "${CMAKE_SOURCE_DIR}/docs/html")
    endif ()
    if (EXISTS "${CMAKE_SOURCE_DIR}/docs/latex")
        file(REMOVE_RECURSE "${CMAKE_SOURCE_DIR}/docs/latex")
    endif ()
    if (EXISTS "${CMAKE_SOURCE_DIR}/html")
        file(REMOVE_RECURSE "${CMAKE_SOURCE_DIR}/html")
    endif ()
    if (EXISTS "${CMAKE_SOURCE_DIR}/latex")
        file(REMOVE_RECURSE "${CMAKE_SOURCE_DIR}/latex")
    endif ()
endif ()