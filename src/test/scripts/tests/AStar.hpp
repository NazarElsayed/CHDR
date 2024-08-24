#ifndef TEST_ASTAR_HPP
#define TEST_ASTAR_HPP

#include <chdr.hpp>

#include <cmath>

#include "../core/Display.hpp"
#include "../generator/Grid.hpp"

namespace Test::Tests {

    class AStar final {

    private:

    public:

        template <typename Tm, const size_t Kd, typename Ts = uint32_t>
        static void Run(const std::array<long unsigned, Kd> _dimensions) {

            // Define some aliases for types to simplify the syntax a little:
            using generator_t = Generator::Grid;
            using     coord_t = CHDR::Coord<size_t, Kd>;
            using    solver_t = CHDR::Solvers::AStar<Tm, Kd, Ts>;
            using   display_t = Display<Tm, Kd>;

            #define HEURISTIC CHDR::Heuristics<Kd, Ts>::ManhattanDistance

            /***************************************/

            // Test parameters:
            constexpr size_t seed(0U);

                      const coord_t size = _dimensions;
            constexpr const coord_t start {};
                      coord_t end;

            constexpr bool checkSolvable = false;

            /* GENERATE MAZE */
            const auto maze = CHDR::Mazes::Grid<Kd, Tm>(size, generator_t::Generate<Tm>(start, end, 0.0F, 0.0F, seed, size));

            /* GRID -> GRAPH */
            //auto graph = CHDR::Mazes::Graph<Tm>(maze);

            /* MAX DRAW SIZE */
            const bool drawable (
                size[0U] <= 100U &&
                size[1U] <= 100U &&
                      Kd >    0U &&
                      Kd <    3U
            );

            /* TEST FOR SOLVABILITY: */
            bool solvable(true);

            std::string solvable_log("\t" + std::string("[SKIPPED] \t(~0ns)"));
            if constexpr (checkSolvable) {

                const auto sw_start = std::chrono::high_resolution_clock::now();

                CHDR::Solvers::FloodFill<Tm, Kd> floodFill;
                solvable = floodFill.Solve(maze, start, end, static_cast<size_t>(std::abs(ceil(CHDR::Heuristics<Kd, Ts>::ManhattanDistance(start, end)))));

                solvable_log = "\t" + std::string(solvable != 0U ? "[SOLVABLE]" : "[IMPOSSIBLE]") + "\t(" +
                    CHDR::Utils::ToString(std::chrono::duration_cast<std::chrono::duration<long double>>(std::chrono::high_resolution_clock::now() - sw_start).count()) + ")";
            }

            // Solve the maze:
            std::string pathfinding_log("\t" + std::string("[SKIPPED] \t(~0ns)"));
            if (solvable) {

                const auto sw_start = std::chrono::high_resolution_clock::now();

                auto solver = solver_t();
                auto path = solver.Solve(maze, start, end, HEURISTIC);

                pathfinding_log = "\t" + std::string(path.size() != 0U ? "[SOLVED]" : "[IMPOSSIBLE]") + "\t(" +
                    CHDR::Utils::ToString(std::chrono::duration_cast<std::chrono::duration<long double>>(std::chrono::high_resolution_clock::now() - sw_start).count()) + ")";

                if (drawable) {
                    display_t::DrawMaze(start, end, size, maze, path);
                }
            }
            else if (drawable) {
                display_t::DrawMaze(start, end, size, maze);
            }

            Debug::Log("(Flood Fill):");
            Debug::Log(solvable_log);

            Debug::Log("(A*):");
            Debug::Log(pathfinding_log);
        }
    };
}

#endif //TEST_ASTAR_HPP