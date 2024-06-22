#include <chdr.hpp>

#include <Debug.hpp>

/**
 * @file main.cpp
 * @brief Global entry point.
 */
int main([[maybe_unused]] int _argc, [[maybe_unused]] char* _argv[]) {

    auto maze = CHDR::Mazes::Grid<2>(32, 32);
    auto solver = CHDR::Solvers::AStar();

    const auto& size = maze.Size();
    Debug::Log(std::to_string(size[0]) + ", " + std::to_string(size[1]));

    auto& node1 = maze.At(0, 0);
    node1.Value(true);

    const auto& node2 = maze.At(0, 0);
    Debug::Log(node2.Value() ? "True" : "False");

    node1.Value(false);
    Debug::Log(node2.Value() ? "True" : "False");

    return 0;
}