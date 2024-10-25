/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#include <../../contrib/LouiEriksson/Debug.hpp>

#include "scripts/core/Application.hpp"

/**
 * @file main.cpp
 * @brief Global entry point.
 */
int main([[maybe_unused]] const int _argc, [[maybe_unused]] const char* _argv[]) {

    int result = -1;

    Debug::Log("CHDR " CHDR_VERSION);
    Debug::Log("Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson");
    Debug::Log("Licensed under CC BY-NC-ND 4.0");
    Debug::Log("main()", Info);

    try {

        switch (_argc) {
            case 1U: {
                break;
            }
            case 2U: {
                result = Test::Application::Main<1U>({
                    std::stoul(_argv[1U])
                });
                break;
            }
            case 3U: {
                result = Test::Application::Main<2U>({
                    std::stoul(_argv[1U]), std::stoul(_argv[2U])
                });
                break;
            }
//            case 4U: {
//                result = Test::Application::Main<3U>({
//                    std::stoul(_argv[1U]), std::stoul(_argv[2U]), std::stoul(_argv[3U])
//                });
//                break;
//            }
//            case 5U: {
//                result = Test::Application::Main<4U>({
//                    std::stoul(_argv[1U]), std::stoul(_argv[2U]), std::stoul(_argv[3U]), std::stoul(_argv[4U])
//                });
//                break;
//            }
//            case 6U: {
//                result = Test::Application::Main<5U>({
//                    std::stoul(_argv[1U]), std::stoul(_argv[2U]), std::stoul(_argv[3U]), std::stoul(_argv[4U]),
//                    std::stoul(_argv[5U])
//                });
//                break;
//            }
//            case 7U: {
//                result = Test::Application::Main<6U>({
//                    std::stoul(_argv[1U]), std::stoul(_argv[2U]), std::stoul(_argv[3U]), std::stoul(_argv[4U]),
//                    std::stoul(_argv[5U]), std::stoul(_argv[6U])
//                });
//                break;
//            }
//            case 8U: {
//                result = Test::Application::Main<7U>({
//                    std::stoul(_argv[1U]), std::stoul(_argv[2U]), std::stoul(_argv[3U]), std::stoul(_argv[4U]),
//                    std::stoul(_argv[5U]), std::stoul(_argv[6U]), std::stoul(_argv[7U])
//                });
//                break;
//            }
//            case 9U: {
//                result = Test::Application::Main<8U>({
//                    std::stoul(_argv[1U]), std::stoul(_argv[2U]), std::stoul(_argv[3U]), std::stoul(_argv[4U]),
//                    std::stoul(_argv[5U]), std::stoul(_argv[6U]), std::stoul(_argv[7U]), std::stoul(_argv[8U])
//                });
//                break;
//            }
//            case 10U: {
//                result = Test::Application::Main<9U>({
//                    std::stoul(_argv[1U]), std::stoul(_argv[2U]), std::stoul(_argv[3U]), std::stoul(_argv[4U]),
//                    std::stoul(_argv[5U]), std::stoul(_argv[6U]), std::stoul(_argv[7U]), std::stoul(_argv[8U]),
//                    std::stoul(_argv[9U])
//                });
//                break;
//            }
//            case 11U: {
//                result = Test::Application::Main<10U>({
//                    std::stoul(_argv[1U]), std::stoul(_argv[ 2U]), std::stoul(_argv[3U]), std::stoul(_argv[4U]),
//                    std::stoul(_argv[5U]), std::stoul(_argv[ 6U]), std::stoul(_argv[7U]), std::stoul(_argv[8U]),
//                    std::stoul(_argv[9U]), std::stoul(_argv[10U])
//                });
//                break;
//            }
//            case 12U: {
//                result = Test::Application::Main<11U>({
//                    std::stoul(_argv[1U]), std::stoul(_argv[ 2U]), std::stoul(_argv[ 3U]), std::stoul(_argv[4U]),
//                    std::stoul(_argv[5U]), std::stoul(_argv[ 6U]), std::stoul(_argv[ 7U]), std::stoul(_argv[8U]),
//                    std::stoul(_argv[9U]), std::stoul(_argv[10U]), std::stoul(_argv[11U])
//                });
//                break;
//            }
//            case 13U: {
//                result = Test::Application::Main<12U>({
//                    std::stoul(_argv[1U]), std::stoul(_argv[ 2U]), std::stoul(_argv[ 3U]), std::stoul(_argv[ 4U]),
//                    std::stoul(_argv[5U]), std::stoul(_argv[ 6U]), std::stoul(_argv[ 7U]), std::stoul(_argv[ 8U]),
//                    std::stoul(_argv[9U]), std::stoul(_argv[10U]), std::stoul(_argv[11U]), std::stoul(_argv[12U])
//                });
//                break;
//            }
//            case 14U: {
//                result = Test::Application::Main<13U>({
//                    std::stoul(_argv[ 1U]), std::stoul(_argv[ 2U]), std::stoul(_argv[ 3U]), std::stoul(_argv[ 4U]),
//                    std::stoul(_argv[ 5U]), std::stoul(_argv[ 6U]), std::stoul(_argv[ 7U]), std::stoul(_argv[ 8U]),
//                    std::stoul(_argv[ 9U]), std::stoul(_argv[10U]), std::stoul(_argv[11U]), std::stoul(_argv[12U]),
//                    std::stoul(_argv[13U])
//                });
//                break;
//            }
//            case 15U: {
//                result = Test::Application::Main<14U>({
//                    std::stoul(_argv[ 1U]), std::stoul(_argv[ 2U]), std::stoul(_argv[ 3U]), std::stoul(_argv[ 4U]),
//                    std::stoul(_argv[ 5U]), std::stoul(_argv[ 6U]), std::stoul(_argv[ 7U]), std::stoul(_argv[ 8U]),
//                    std::stoul(_argv[ 9U]), std::stoul(_argv[10U]), std::stoul(_argv[11U]), std::stoul(_argv[12U]),
//                    std::stoul(_argv[13U]), std::stoul(_argv[14U])
//                });
//                break;
//            }
//            case 16U: {
//                result = Test::Application::Main<15U>({
//                    std::stoul(_argv[ 1U]), std::stoul(_argv[ 2U]), std::stoul(_argv[ 3U]), std::stoul(_argv[ 4U]),
//                    std::stoul(_argv[ 5U]), std::stoul(_argv[ 6U]), std::stoul(_argv[ 7U]), std::stoul(_argv[ 8U]),
//                    std::stoul(_argv[ 9U]), std::stoul(_argv[10U]), std::stoul(_argv[11U]), std::stoul(_argv[12U]),
//                    std::stoul(_argv[13U]), std::stoul(_argv[14U]), std::stoul(_argv[15U])
//                });
//                break;
//            }
//            case 17U: {
//                result = Test::Application::Main<16U>({
//                    std::stoul(_argv[ 1U]), std::stoul(_argv[ 2U]), std::stoul(_argv[ 3U]), std::stoul(_argv[ 4U]),
//                    std::stoul(_argv[ 5U]), std::stoul(_argv[ 6U]), std::stoul(_argv[ 7U]), std::stoul(_argv[ 8U]),
//                    std::stoul(_argv[ 9U]), std::stoul(_argv[10U]), std::stoul(_argv[11U]), std::stoul(_argv[12U]),
//                    std::stoul(_argv[13U]), std::stoul(_argv[14U]), std::stoul(_argv[15U]), std::stoul(_argv[16U])
//                });
//                break;
//            }
            default: {
                throw std::runtime_error("ERROR: Invalid Dimensionality! Please use between 2 and 16 arguments.");
            }
        }
    }
    catch (const std::exception& e) {
        Debug::Log(e);
    }

    return result;
}