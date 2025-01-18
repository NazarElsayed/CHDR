# Computational Helper for Direction and Routing (CHDR)
# Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
#
# Licensed under CC BY-NC-ND 4.0
# https://creativecommons.org/licenses/by-nc-nd/4.0/

cmake_minimum_required(VERSION ${CMAKE_VERSION})

project(CHDR VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

if (DEFINED SUPER_PEDANTIC)
    message(WARNING "WARNING: SUPER_PEDANTIC enabled. Build system may catch warnings as errors.")

    set(WARNINGS "-pedantic -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Wnon-virtual-dtor -Wold-style-cast -Wcast-align -Woverloaded-virtual -Wnull-dereference -Wdouble-promotion -Wvla -Wformat=2 -Wmissing-declarations -Wsuggest-override -Wstrict-overflow=5 -Wuninitialized -Wswitch-enum -Wswitch-default -Wcast-qual -Wwrite-strings -Wpointer-arith -Wfloat-equal -Wc++20-extensions -Werror")
else()
    set(WARNINGS "-Wall -Wextra -Wpedantic -Wconversion -Wshadow -Wnon-virtual-dtor -Wold-style-cast -Wcast-align -Woverloaded-virtual -Wnull-dereference -Wdouble-promotion -Wvla -Wformat=2 -Wmissing-declarations -Wsuggest-override -Wstrict-overflow=5 -Wuninitialized -Wswitch-enum -Wswitch-default -Wcast-qual -Wwrite-strings -Wpointer-arith -Wfloat-equal -Wc++20-extensions")
endif()

# DIFFERENT BUILD MODES:
set(CMAKE_CXX_FLAGS_DEBUG        "${WARNINGS} -O0 -fsanitize=address,leak,undefined -fno-omit-frame-pointer -fno-rtti -g")
set(CMAKE_CXX_FLAGS_NOSANITIZERS "${WARNINGS} -O0                                   -fno-omit-frame-pointer -fno-rtti -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${WARNINGS} -O2 -flto -march=native -mtune=native -funroll-loops -ftree-vectorize -fstrict-aliasing -fomit-frame-pointer -fno-rtti -DNDEBUG")

set(CHDR_VERSION_TWEAK "alpha")
if (CHDR_VERSION_TWEAK)
    set(CHDR_VERSION "${CHDR_VERSION_MAJOR}.${CHDR_VERSION_MINOR}.${CHDR_VERSION_PATCH}-${CHDR_VERSION_TWEAK}")
endif ()

set(CHDR_VERSION_METADATA "${CMAKE_SYSTEM_PROCESSOR}-${CMAKE_SYSTEM_NAME}")

if (CMAKE_CXX_FLAGS_RELEASE MATCHES "-march=native|-mtune=native|-mcpu=native")
    set(CHDR_VERSION_METADATA "${CHDR_VERSION_METADATA}-native")
endif ()

if (CHDR_VERSION_METADATA)
    set(CHDR_VERSION "${CHDR_VERSION}+${CHDR_VERSION_METADATA}")
endif ()

configure_file(chdr_version.hpp.in ${CMAKE_CURRENT_SOURCE_DIR}/chdr_version.hpp)

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