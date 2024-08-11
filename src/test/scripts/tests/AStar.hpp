#ifndef TEST_ASTAR_HPP
#define TEST_ASTAR_HPP

#include <chdr.hpp>

#include "../core/Display.hpp"
#include "../generator/Grid.hpp"

namespace Test::Tests {

    class AStar final {

    public:

        template <typename Tm, const size_t Kd, typename Ts = uint32_t>
        static void Run() {

            // Define some aliases for types to simplify the syntax a little:
            using generator_t = Generator::Grid;
            using     coord_t = CHDR::Coord<size_t, Kd>;
            using    solver_t = CHDR::Solvers::AStar<Tm, Kd, Ts>;
            using   display_t = Display<Tm, Kd>;

            #define HEURISTIC CHDR::Heuristics<Kd, Ts>::ManhattanDistance

            /***************************************/

            // Generate a maze:
            coord_t size  { 10000U, 10000U };
            coord_t start {    0U,    0U };
            coord_t end;

            size_t seed = 0U;

            auto nodes = generator_t::Generate<Tm>(start, end, 0.0F, 0.0F, seed, size);
            auto maze = CHDR::Mazes::HeavyGrid<Kd, Tm>(size, nodes);

            Debug::Log("Maze Generated!");

            // Solve the maze:
            //if (solver_t::HasSolution(maze, start, end, static_cast<size_t>(std::abs(ceil(CHDR::Heuristics<Kd, Ts>::ManhattanDistance(start, end)))))) {

            Debug::Log("Flood fill done!", Debug);

            auto solver = solver_t();
            auto path = solver.Solve(maze, start, end, HEURISTIC);

            Debug::Log("Solved!");

            display_t::DrawMaze(start, end, size, maze, path);
            // }
            // else {
            //     display_t::DrawMaze(start, end, size, maze);
            // }
        }
    };
}

#endif //TEST_ASTAR_HPP