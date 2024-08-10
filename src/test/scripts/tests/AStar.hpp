#ifndef TEST_ASTAR_HPP
#define TEST_ASTAR_HPP

#include <chdr.hpp>

#include "../core/Display.hpp"
#include "../generator/Grid.hpp"

namespace Test::Tests {

    class AStar final {

    private:

        static std::string Trim_Trailing_Zeros(std::string _str) {

            _str.erase(_str.find_last_not_of('0') + 1, std::string::npos);

            if (_str.back() == '.') {
                _str.pop_back();
            }

            return _str;
        }

        static std::string ToString(long double _duration) {

            static std::array<std::string, 4> units = { "s", "ms", "µs", "ns" };

            size_t i = 0U;
            while (i < units.size() && _duration < 1.0) {
                _duration *= 1000.0;
                ++i;
            }

            return Trim_Trailing_Zeros(std::to_string(_duration)) + units[i];
        }

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

            // Test parameters:
            constexpr size_t seed = 0U;

            constexpr coord_t size  { 10000U, 10000U };
            constexpr coord_t start {};
                      coord_t end;

            constexpr bool checkSolvable = false;

            /* GENERATE MAZE */
            const auto maze = CHDR::Mazes::Grid<Kd, Tm>(size, generator_t::Generate<Tm>(start, end, 0.0F, 0.0F, seed, size));

            Debug::Log("Maze: [GENERATED]");

            /* GRID -> GRAPH */
            //auto graph = CHDR::Mazes::Graph<Tm>(size, generator_t::Generate<Tm>(start, end, 0.0F, 0.0F, seed, size));

            /* MAX DRAW SIZE */
            bool drawable (
                            size[0] <= 100U &&
                (Kd >= 1 || size[1] <= 100U)
            );

            /* TEST FOR SOLVABILITY: */
            bool solvable(true);

            std::string solvable_log("(Flood Fill): \t" + std::string("[SKIPPED] \t(~0ns)"));
            if constexpr (checkSolvable) {

                const auto sw_start = std::chrono::high_resolution_clock::now();

                solvable = solver_t::HasSolution(maze, start, end, static_cast<size_t>(std::abs(ceil(CHDR::Heuristics<Kd, Ts>::ManhattanDistance(start, end)))));

                solvable_log = "(Flood Fill): \t" + std::string(solvable != 0U ? "[SOLVABLE]" : "[IMPOSSIBLE]") + "\t(" +
                    ToString(std::chrono::duration_cast<std::chrono::duration<long double>>(std::chrono::high_resolution_clock::now() - sw_start).count()) + ")";
            }

            // Solve the maze:
            std::string pathfinding_log("(A*): \t\t\t" + std::string("[SKIPPED] \t(~0ns)"));
            if (solvable) {

                const auto sw_start = std::chrono::high_resolution_clock::now();

                auto solver = solver_t();
                auto path = solver.Solve(maze, start, end, HEURISTIC);

                pathfinding_log = "(A*): \t\t\t" + std::string(path.size() != 0U ? "[SOLVED]" : "[IMPOSSIBLE]") + "\t(" +
                    ToString(std::chrono::duration_cast<std::chrono::duration<long double>>(std::chrono::high_resolution_clock::now() - sw_start).count()) + ")";

                if (drawable) {
                    display_t::DrawMaze(start, end, size, maze, path);
                }
            }
            else if (drawable) {
                display_t::DrawMaze(start, end, size, maze);
            }

            Debug::Log(Trim_Trailing_Zeros(std::to_string(CHDR::Utils::Product<size_t>(size) / static_cast<long double>(1000000000.0))) + "b total candidate nodes.");
            Debug::Log(   solvable_log);
            Debug::Log(pathfinding_log);
        }
    };
}

#endif //TEST_ASTAR_HPP