#ifndef TEST_ASTAR_HPP
#define TEST_ASTAR_HPP

#include <chdr.hpp>

#include "../core/Display.hpp"
#include "../generator/Grid.hpp"

namespace Test::Tests {

    class AStar final {

    public:

        template <typename Tm, size_t Kd, typename Ts = uint16_t>
        static void Run() {

            // Define some aliases for types to simplify the syntax a little:
            using generator_t = Generator::Grid;
            using     coord_t = CHDR::Coord<size_t, Kd>;
            using    solver_t = CHDR::Solvers::AStar<Tm, Kd, Ts>;
            using   display_t = Display<Tm, Kd>;

            /***************************************/

            // Generate a maze:
            coord_t size  { 32U, 32U };
            coord_t start {  0U,  0U };
            coord_t end;

            size_t seed = 0U;

            const auto maze = CHDR::Mazes::Grid<Kd, Tm>(size, generator_t::Generate<Tm>(start, end, seed, size));

            Debug::Log("Maze Generated!");

            // Solve the maze:
            if (solver_t::HasSolution(maze, start, end, static_cast<size_t>(ceil(solver_t::ManhattanDistance(start, end))))) {

                Debug::Log("Flood fill done!", Debug);

                auto solver = solver_t();
                auto path = solver.Solve(maze, start, end, solver_t::ManhattanDistance);

                Debug::Log("Solved!");

                display_t::DrawMaze(start, end, size, maze, path);
            }
            else {
                display_t::DrawMaze(start, end, size, maze);
            }
        }
    };
}

#endif //TEST_ASTAR_HPP