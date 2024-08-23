#include <../../contrib/LouiEriksson/Debug.hpp>

#include "scripts/core/Application.hpp"

/**
 * @file main.cpp
 * @brief Global entry point.
 */
int main([[maybe_unused]] const int _argc, [[maybe_unused]] const char* _argv[]) {

    int exitCode = 0;

    Debug::Log("CHDR Version: v" CHDR_VERSION, Info);

    if (_argc > 2) {
        Debug::Log("main()", Info);

        const std::array<long unsigned, 2U> dimensions = {std::stoul(_argv[1]), std::stoul(_argv[2])};
        exitCode = Test::Application::Main(dimensions);
    }

    return exitCode;
}