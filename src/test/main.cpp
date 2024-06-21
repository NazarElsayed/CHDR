#include <chdr.hpp>

#include <Debug.hpp>

/**
 * @file main.cpp
 * @brief Global entry point.
 */
int main([[maybe_unused]] int _argc, [[maybe_unused]] char* _argv[]) {

    auto maze = CHDR::Mazes::Grid();
    auto solver = CHDR::Solvers::AStar();

    Debug::Log("Hello World");

    return 0;
}