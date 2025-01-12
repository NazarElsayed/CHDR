/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#include <chdr.hpp>

#include <debug.hpp>

#include <exception>
#include <stdexcept>
#include <string>

#include "core/application.hpp"

template <typename coord_t>
static constexpr int execute(const std::string_view& _solver, const coord_t& _size);

/**
 * @file main.cpp
 * @brief Global entry point.
 */
int main(int _argc, const char* const _argv[]) noexcept {

    int result = -1;

    debug::log("CHDR " CHDR_VERSION);
    debug::log("Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson");
    debug::log("Licensed under CC BY-NC-ND 4.0");
    debug::log("main()", info);

    using index_t = unsigned long;

         if (_argc == 3U) { result = test::application::main( { _argv[1U] }, chdr::coord<index_t, 1U> { std::stoul(_argv[2U]) }); }
    else if (_argc == 4U) { result = test::application::main( { _argv[1U] }, chdr::coord<index_t, 2U> { std::stoul(_argv[2U]), std::stoul(_argv[3U]) }); }
    else if (_argc == 5U) { result = test::application::main( { _argv[1U] }, chdr::coord<index_t, 3U> { std::stoul(_argv[2U]), std::stoul(_argv[3U]), std::stoul(_argv[4U]) }); }
    else if (_argc == 6U) { result = test::application::main( { _argv[1U] }, chdr::coord<index_t, 4U> { std::stoul(_argv[2U]), std::stoul(_argv[3U]), std::stoul(_argv[4U]), std::stoul(_argv[5U]) }); }
    else {
        debug::log("ERROR: Invalid Dimensionality! Please use between 1 and 4 arguments.", error);
    }

    return result;
}