#ifndef TEST_ASTAR_HPP
#define TEST_ASTAR_HPP

#include <chdr.hpp>

#include "../core/Display.hpp"
#include "../generator/Grid.hpp"

namespace Test::Tests {

    class AStar final {

    public:

        template <typename Tm, size_t Kd, typename Ts = float>
        static void Run() {

            // Define some aliases for types to simplify the syntax a little:
            using Generator = Generator::Grid;
            using coord_t   = CHDR::Coord<size_t, Kd>;

            /***************************************/

            // Generate a maze:
            coord_t size  { 1000U, 1000U };
            coord_t start {    0U,    0U };
            coord_t end;

            size_t seed = 0U;

            const auto maze = CHDR::Mazes::Grid<Kd, Tm>(size, Generator::Generate<Tm>(start, end, seed, size));

            // Solve the maze:
            if (CHDR::Solvers::AStar<Tm, Kd>().HasSolution(maze, start, end)) {

                auto solver = CHDR::Solvers::AStar<Tm, Kd, int>();
                auto path = solver.Solve(maze, start, end, CHDR::Solvers::AStar<Tm, Kd, int>::ManhattanDistance);

                Display<Tm, Kd>::DrawMaze(start, end, size, maze, path);
            }
            else {
                Display<Tm, Kd>::DrawMaze(start, end, size, maze);
            }
        }
    };
}

#endif //TEST_ASTAR_HPP