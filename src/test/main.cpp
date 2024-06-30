#include <../../contrib/LouiEriksson/Debug.hpp>

#include "scripts/core/Application.hpp"

/**
 * @file main.cpp
 * @brief Global entry point.
 */
int main([[maybe_unused]] int _argc, [[maybe_unused]] char* _argv[]) {

    Debug::Log("main()", Info);

    return Test::Application::Main();
}