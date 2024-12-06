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

#include "../core/display.hpp"
#include "../generator/graph.hpp"
#include "../generator/grid.hpp"

namespace test::tests {

    class astar final {

    private:

    public:

        template <typename weight_t, const size_t Kd, typename scalar_t = uint32_t, typename index_t = uint32_t>
        static void run(const std::array<index_t, Kd>& _dimensions) {

            #define HEURISTIC chdr::heuristics<Kd, scalar_t>::manhattan_distance

            using coord_t = chdr::coord<index_t, Kd>;

            // Test parameters:
            constexpr size_t seed(0U);

            const     coord_t size = _dimensions;
            constexpr coord_t start {};
                      coord_t end;

            /* GENERATE MAZE */
            const auto grid = generator::grid::generate<weight_t>(start, end, 0.0, 0.0, seed, size);
            const auto graph = chdr::mazes::graph<index_t, scalar_t>(grid);
            //const auto graph = generator::graph::generate<weight_t, index_t, scalar_t>(start, end, 0.0, 0.0, seed, size);

            /* MAX DRAW SIZE */
            const bool drawable (
                size[0U] <= 100U &&
                size[1U] <= 100U &&
                      Kd >    0U &&
                      Kd <    3U
            );

            const auto sw_start = std::chrono::high_resolution_clock::now();

            auto solver = chdr::solvers::astar<weight_t, Kd, scalar_t, index_t>();
            // auto path = solver.solve(grid, start, end, HEURISTIC);
            auto path = solver.solve(graph, start, end, size, HEURISTIC);

            const auto pathfinding_log = "\t" + std::string(path.size() != 0U ? "[SOLVED]" : "[IMPOSSIBLE]") + "\t(~" +
                chdr::utils::to_string(std::chrono::duration_cast<std::chrono::duration<long double>>(std::chrono::high_resolution_clock::now() - sw_start).count()) + ")";

            if (drawable) {
                display<weight_t, Kd>::draw_maze(start, end, size, grid, path);
            }

            debug::log("(A*):");
            debug::log(pathfinding_log);

            #undef HEURISTIC
        }
    };

} // test::tests

#endif //TEST_ASTAR_HPP