/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

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
        static void Run(const std::array<size_t, Kd>& _dimensions) {

            #define HEURISTIC CHDR::Heuristics<Kd, Ts>::ManhattanDistance

            using coord_t = CHDR::Coord<size_t, Kd>;

            // Test parameters:
            constexpr size_t seed(0U);

                      const coord_t size = _dimensions;
            constexpr const coord_t start {};
                      coord_t end;

            /* GENERATE MAZE */
            const auto grid = CHDR::Mazes::Grid<Kd, Tm>(size, Generator::Grid::Generate<Tm>(start, end, 0.0F, 0.0F, seed, size));

            /* GRID -> GRAPH */
            Debug::Log("(Graph):");
            auto graph = CHDR::Mazes::Graph<size_t, Kd, Ts>(grid);
            Debug::Log("\t[GENERATED] (" + std::to_string(graph.Count()) + " candidate nodes)");
            graph.Prune();
            Debug::Log("\t[PRUNED] (" + std::to_string(graph.Count()) + " candidate nodes)");

            /* MAX DRAW SIZE */
            const bool drawable (
                size[0U] <= 100U &&
                size[1U] <= 100U &&
                      Kd >    0U &&
                      Kd <    3U
            );

            const auto sw_start = std::chrono::high_resolution_clock::now();

            auto solver = CHDR::Solvers::JPS<Tm, Kd, Ts>();
            auto path = solver.Solve(grid, start, end, HEURISTIC);

            auto pathfinding_log = "\t" + std::string(path.size() != 0U ? "[SOLVED]" : "[IMPOSSIBLE]") + "\t(~" +
                CHDR::Utils::ToString(std::chrono::duration_cast<std::chrono::duration<long double>>(std::chrono::high_resolution_clock::now() - sw_start).count()) + ")";

            if (drawable) {
                Display<Tm, Kd>::DrawMaze(start, end, size, grid, path);
            }

            Debug::Log("(A*):");
            Debug::Log(pathfinding_log);

            #undef HEURISTIC
        }
    };
}

#endif //TEST_ASTAR_HPP