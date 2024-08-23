#include <../../contrib/LouiEriksson/Debug.hpp>

#include "scripts/core/Application.hpp"

/**
 * @file main.cpp
 * @brief Global entry point.
 */
int main([[maybe_unused]] const int _argc, [[maybe_unused]] const char* _argv[]) {

    int exitCode = 0;

    Debug::Log("CHDR Version: v" CHDR_VERSION, Info);

    if (_argc > 2U) {

        Debug::Log("main()", Info);

        switch (_argc) {
            case 2U: {
                exitCode = Test::Application::Main<1U>({std::stoul(_argv[1U])});
                break;
            }
            case 3U: {
                exitCode = Test::Application::Main<2U>({std::stoul(_argv[1U]), std::stoul(_argv[2U])});
                break;
            }
            case 4U: {
                exitCode = Test::Application::Main<3U>({
                    std::stoul(_argv[1U]), std::stoul(_argv[2U]), std::stoul(_argv[3U])
                });
                break;
            }
            case 5U: {
                exitCode = Test::Application::Main<4U>({
                    std::stoul(_argv[1U]), std::stoul(_argv[2U]), std::stoul(_argv[3U]), std::stoul(_argv[4U])
                });
                break;
            }
            case 6U: {
                exitCode = Test::Application::Main<5U>({
                    std::stoul(_argv[1U]), std::stoul(_argv[2U]), std::stoul(_argv[3U]), std::stoul(_argv[4U]),
                    std::stoul(_argv[5U])
                });
                break;
            }
            case 7U: {
                exitCode = Test::Application::Main<6U>({
                    std::stoul(_argv[1U]), std::stoul(_argv[2U]), std::stoul(_argv[3U]), std::stoul(_argv[4U]),
                    std::stoul(_argv[5U]), std::stoul(_argv[6U])
                });
                break;
            }
            case 8U: {
                exitCode = Test::Application::Main<7U>({
                    std::stoul(_argv[1U]), std::stoul(_argv[2U]), std::stoul(_argv[3U]), std::stoul(_argv[4U]),
                    std::stoul(_argv[5U]), std::stoul(_argv[6U]), std::stoul(_argv[7U])
                });
                break;
            }
            case 9U: {
                exitCode = Test::Application::Main<8U>({
                    std::stoul(_argv[1U]), std::stoul(_argv[2U]), std::stoul(_argv[3U]), std::stoul(_argv[4U]),
                    std::stoul(_argv[5U]), std::stoul(_argv[6U]), std::stoul(_argv[7U]), std::stoul(_argv[8U])
                });
                break;
            }
            case 10U: {
                exitCode = Test::Application::Main<9U>({
                    std::stoul(_argv[1U]), std::stoul(_argv[2U]), std::stoul(_argv[3U]), std::stoul(_argv[4U]),
                    std::stoul(_argv[5U]), std::stoul(_argv[6U]), std::stoul(_argv[7U]), std::stoul(_argv[8U]),
                    std::stoul(_argv[9U])
                });
                break;
            }
            case 11U: {
                exitCode = Test::Application::Main<10U>({
                    std::stoul(_argv[1U]), std::stoul(_argv[2U]), std::stoul(_argv[3U]), std::stoul(_argv[4U]),
                    std::stoul(_argv[5U]), std::stoul(_argv[6U]), std::stoul(_argv[7U]), std::stoul(_argv[8U]),
                    std::stoul(_argv[9U]), std::stoul(_argv[10U])
                });
                break;
            }
            case 12U: {
                exitCode = Test::Application::Main<11U>({
                    std::stoul(_argv[1U]), std::stoul(_argv[2U]), std::stoul(_argv[3U]), std::stoul(_argv[4U]),
                    std::stoul(_argv[5U]), std::stoul(_argv[6U]), std::stoul(_argv[7U]), std::stoul(_argv[8U]),
                    std::stoul(_argv[9U]), std::stoul(_argv[10U]), std::stoul(_argv[11U])
                });
                break;
            }
            case 13U: {
                exitCode = Test::Application::Main<12U>({
                    std::stoul(_argv[1U]), std::stoul(_argv[2U]), std::stoul(_argv[3U]), std::stoul(_argv[4U]),
                    std::stoul(_argv[5U]), std::stoul(_argv[6U]), std::stoul(_argv[7U]), std::stoul(_argv[8U]),
                    std::stoul(_argv[9U]), std::stoul(_argv[10U]), std::stoul(_argv[11U]), std::stoul(_argv[12U])
                });
                break;
            }
            case 14U: {
                exitCode = Test::Application::Main<13U>({
                    std::stoul(_argv[1U]), std::stoul(_argv[2U]), std::stoul(_argv[3U]), std::stoul(_argv[4U]),
                    std::stoul(_argv[5U]), std::stoul(_argv[6U]), std::stoul(_argv[7U]), std::stoul(_argv[8U]),
                    std::stoul(_argv[9U]), std::stoul(_argv[10U]), std::stoul(_argv[11U]), std::stoul(_argv[12U]),
                    std::stoul(_argv[13U])
                });
                break;
            }
            case 15U: {
                exitCode = Test::Application::Main<14U>({
                    std::stoul(_argv[1U]), std::stoul(_argv[2U]), std::stoul(_argv[3U]), std::stoul(_argv[4U]),
                    std::stoul(_argv[5U]), std::stoul(_argv[6U]), std::stoul(_argv[7U]), std::stoul(_argv[8U]),
                    std::stoul(_argv[9U]), std::stoul(_argv[10U]), std::stoul(_argv[11U]), std::stoul(_argv[12U]),
                    std::stoul(_argv[13U]), std::stoul(_argv[14U])
                });
                break;
            }
            case 16U: {
                exitCode = Test::Application::Main<15U>({
                    std::stoul(_argv[1U]), std::stoul(_argv[2U]), std::stoul(_argv[3U]), std::stoul(_argv[4U]),
                    std::stoul(_argv[5U]), std::stoul(_argv[6U]), std::stoul(_argv[7U]), std::stoul(_argv[8U]),
                    std::stoul(_argv[9U]), std::stoul(_argv[10U]), std::stoul(_argv[11U]), std::stoul(_argv[12U]),
                    std::stoul(_argv[13U]), std::stoul(_argv[14U]), std::stoul(_argv[15U])
                });
                break;
            }
            case 17U: {
                exitCode = Test::Application::Main<16U>({
                    std::stoul(_argv[1U]), std::stoul(_argv[2U]), std::stoul(_argv[3U]), std::stoul(_argv[4U]),
                    std::stoul(_argv[5U]), std::stoul(_argv[6U]), std::stoul(_argv[7U]), std::stoul(_argv[8U]),
                    std::stoul(_argv[9U]), std::stoul(_argv[10U]), std::stoul(_argv[11U]), std::stoul(_argv[12U]),
                    std::stoul(_argv[13U]), std::stoul(_argv[14U]), std::stoul(_argv[15U]), std::stoul(_argv[16U])
                });
                break;
            }
            default: {
                throw std::runtime_error(
                    "Runtime testing has a limit of no more than 16 dimensions. "
                    "Include CHDR in your own program to extend this limit."
                );

                exitCode = 69;
            }
        }
    }

    return exitCode;
}