#include <chdr.hpp>

#include <Debug.hpp>

/**
 * @file main.cpp
 * @brief Global entry point.
 */
int main([[maybe_unused]] int _argc, [[maybe_unused]] char* _argv[]) {

    auto maze = CHDR::Mazes::Grid<2>();
    auto solver = CHDR::Solvers::AStar();

    static constexpr size_t index1D = 63;

    static constexpr auto asNd = CHDR::Utils::ToND(index1D, 4, 4, 4);
    std::cout << index1D << " is: (" << asNd[0] << ", " << asNd[1] << ", " << asNd[2] << ")\n";

    static constexpr auto as1D = CHDR::Utils::To1D(asNd, 4, 4, 4);
    std::cout << "(" << asNd[0] << ", " << asNd[1] << ", " << asNd[2] << ") is: " << as1D << "\n";

    Debug::Log("Hello World");

    return 0;
}