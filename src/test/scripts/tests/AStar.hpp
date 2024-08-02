#ifndef TEST_ASTAR_HPP
#define TEST_ASTAR_HPP

#include <chdr.hpp>

#include "../core/Display.hpp"
#include "../generator/Grid.hpp"

namespace Test::Tests {

    class AStar final {

    public:

        template <typename T, size_t Kd, typename Ts = float>
        static void Run() {

            // Define some aliases for types to simplify the syntax a little:
            using Generator = Generator::Grid;
            using coord_t   = CHDR::Coord<size_t, Kd>;

            /***************************************/

            // Generate a maze:
            coord_t size  { 32U, 32U };
            coord_t start {  0U,  0U };
            coord_t end;

            size_t seed = 0U;

            const auto maze = CHDR::Mazes::Grid<Kd, T>(size, Generator::Generate<T>(start, end, seed, size));

            // Solve the maze:
            if (CHDR::Solvers::AStar<T, Kd>().HasSolution(maze, start, end)) {

                auto solver = CHDR::Solvers::AStar<T, Kd, Ts>();
                auto path = solver.Solve(maze, start, end);

                Display<T, Kd>::DrawMaze(start, end, size, maze, path);
            }
            else {
                Display<T, Kd>::DrawMaze(start, end, size, maze);
            }
        }
    };
}

#endif //TEST_ASTAR_HPP