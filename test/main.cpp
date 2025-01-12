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

/**
 * @file main.cpp
 * @brief Global entry point.
 */
int main([[maybe_unused]] const int _argc, [[maybe_unused]] const char* _argv[]) noexcept {

    int result = -1;

    debug::log("CHDR " CHDR_VERSION);
    debug::log("Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson");
    debug::log("Licensed under CC BY-NC-ND 4.0");
    debug::log("main()", info);

    try {
        using T = unsigned long;

        switch (_argc) {
            case 1U: {
                break;
            }
            case 2U: {
                result = test::application::main( chdr::coord<T, 1U> {
                    std::stoul(_argv[1U])
                });
                break;
            }
            case 3U: {
                result = test::application::main( chdr::coord<T, 2U> {
                    std::stoul(_argv[1U]), std::stoul(_argv[2U])
                });
                break;
            }
            case 4U: {
                result = test::application::main( chdr::coord<T, 3U> {
                    std::stoul(_argv[1U]), std::stoul(_argv[2U]), std::stoul(_argv[3U])
                });
                break;
            }
            case 5U: {
                result = test::application::main( chdr::coord<T, 4U> {
                    std::stoul(_argv[1U]), std::stoul(_argv[2U]), std::stoul(_argv[3U]), std::stoul(_argv[4U])
                });
                break;
            }
            case 6U: {
                result = test::application::main( chdr::coord<T, 5U> {
                    std::stoul(_argv[1U]), std::stoul(_argv[2U]), std::stoul(_argv[3U]), std::stoul(_argv[4U]),
                    std::stoul(_argv[5U])
                });
                break;
            }
            case 7U: {
                result = test::application::main( chdr::coord<T, 6U> {
                    std::stoul(_argv[1U]), std::stoul(_argv[2U]), std::stoul(_argv[3U]), std::stoul(_argv[4U]),
                    std::stoul(_argv[5U]), std::stoul(_argv[6U])
                });
                break;
            }
            case 8U: {
                result = test::application::main( chdr::coord<T, 7U> {
                    std::stoul(_argv[1U]), std::stoul(_argv[2U]), std::stoul(_argv[3U]), std::stoul(_argv[4U]),
                    std::stoul(_argv[5U]), std::stoul(_argv[6U]), std::stoul(_argv[7U])
                });
                break;
            }
            case 9U: {
                result = test::application::main( chdr::coord<T, 8U> {
                    std::stoul(_argv[1U]), std::stoul(_argv[2U]), std::stoul(_argv[3U]), std::stoul(_argv[4U]),
                    std::stoul(_argv[5U]), std::stoul(_argv[6U]), std::stoul(_argv[7U]), std::stoul(_argv[8U])
                });
                break;
            }
            default: {
                throw std::runtime_error("ERROR: Invalid Dimensionality! Please use between 2 and 8 arguments.");
            }
        }
    }
    catch (const std::exception& e) {
        debug::log(e);
    }

    return result;
}