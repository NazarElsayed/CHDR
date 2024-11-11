/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef TEST_ASTAR_HPP
#define TEST_ASTAR_HPP

#include <cmath>

#include <chdr.hpp>

#include "../core/Display.hpp"
#include "../generator/Grid.hpp"
#include "../generator/Graph.hpp"

namespace Test::Tests {

    class AStar final {

    private:

    public:

        template <typename weight_t, const size_t Kd, typename scalar_t = uint32_t, typename index_t = uint32_t>
        static void Run(const std::array<index_t, Kd>& _dimensions) {

            #define HEURISTIC CHDR::Heuristics<Kd, scalar_t>::ManhattanDistance

            using coord_t = CHDR::Coord<index_t, Kd>;

            // Test parameters:
            constexpr size_t seed(0U);

                      const coord_t size = _dimensions;
            constexpr const coord_t start {};
                      coord_t end;

            /* GENERATE MAZE */
            const auto grid = Generator::Grid::Generate<weight_t>(start, end, 0.0, 0.0, seed, size);
            //const auto graph = Generator::Graph::Generate<weight_t, index_t, scalar_t>(start, end, 0.0, 0.0, seed, size);

            /* MAX DRAW SIZE */
            const bool drawable (
                size[0U] <= 100U &&
                size[1U] <= 100U &&
                      Kd >    0U &&
                      Kd <    3U
            );

            const auto graph = CHDR::Mazes::Graph<index_t, scalar_t>(grid);

            const auto sw_start = std::chrono::high_resolution_clock::now();

            auto solver = CHDR::Solvers::DFS<weight_t, Kd, scalar_t, index_t>();
//            auto path = solver.Solve(grid, start, end, HEURISTIC);
            auto path = solver.Solve(graph, start, end, size, HEURISTIC);

            auto pathfinding_log = "\t" + std::string(path.size() != 0U ? "[SOLVED]" : "[IMPOSSIBLE]") + "\t(~" +
                CHDR::Utils::ToString(std::chrono::duration_cast<std::chrono::duration<long double>>(std::chrono::high_resolution_clock::now() - sw_start).count()) + ")";

            if (drawable) {
                Display<weight_t, Kd>::DrawMaze(start, end, size, grid, path);
            }

            Debug::Log("(A*):");
            Debug::Log(pathfinding_log);

            #undef HEURISTIC
        }
    };
}

#endif //TEST_ASTAR_HPP