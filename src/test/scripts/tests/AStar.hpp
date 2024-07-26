#ifndef TEST_ASTAR_HPP
#define TEST_ASTAR_HPP

#include <chdr.hpp>

#include "../generator/Grid.hpp"

namespace Test::Tests {

    class AStar final {

    public:

        template <typename T, size_t Kd>
        static void Run() {

            // Define some aliases for types to simplify the syntax a little:
            using Generator = Generator::Grid;
            using coord_t   = CHDR::Coord<size_t, Kd>;

            /***************************************/

            coord_t size  { 100U, 100U };
            coord_t start {  0U,  0U };
            coord_t end;

            // Generate a maze:
            auto maze = CHDR::Mazes::Grid<Kd, T>(size);
            maze.Nodes(Generator::Generate<T>(start, end, 0U, size));

            auto& start1 = maze.At(start);
            start1.Value(true);

            auto& start2 = maze.At(start);
            start2.Value(false);

            Debug::Log(std::to_string(start1.Value()) + " == " + std::to_string(start2.Value()));

            // Display the maze:
            Generator::Display(start, end, size, maze);

            // Solve the maze:
            auto solver = CHDR::Solvers::AStar<T, Kd>();
            solver.Solve(maze, start, end);
        }
    };
}

#endif //TEST_ASTAR_HPP