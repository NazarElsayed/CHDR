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

#include <debug.hpp>

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>

#include "../core/display.hpp"

// ReSharper disable CppUnusedIncludeDirective

#include "../generator/graph.hpp"   // NOLINT(*-include-cleaner)
#include "../generator/grid.hpp"    // NOLINT(*-include-cleaner)

// ReSharper restore CppUnusedIncludeDirective

namespace test::tests {

    class astar final {

    private:

    public:

        template <typename weight_t, size_t Kd, typename scalar_t = uint32_t, typename index_t = uint32_t>
        static void run(const std::array<index_t, Kd>& _dimensions) {

            #define HEURISTIC chdr::heuristics<Kd, scalar_t>::manhattan_distance

            using coord_t = chdr::coord<index_t, Kd>;

            // Test parameters:
            const size_t test_samples = std::max(
#ifndef NDEBUG
                1000000UL
#else //!NDEBUG
                100000000UL
#endif //!NDEBUG
                / chdr::utils::product<size_t>(_dimensions),
                1UL
            );

            constexpr size_t seed(0U);

            const     coord_t size = _dimensions;
            constexpr coord_t start {};
                      coord_t end;

            /* MAX DRAW SIZE */
            const bool drawable (
                size[0U] <= 100U &&
                size[1U] <= 100U &&
                      Kd >    0U &&
                      Kd <    3U
            );

            /* GENERATE MAZE */

            const auto maze = generator::grid::generate<weight_t>(start, end, 0.0, 0.0, seed, size);
            //const auto maze = chdr::mazes::graph<index_t, scalar_t>(generator::grid::generate<weight_t>(start, end, 0.0, 0.0, seed, size));
            //const auto maze = generator::graph::generate<weight_t, index_t, scalar_t>(start, end, 0.0, 0.0, seed, size);

            /* CAPTURE SYSTEM NOISE */

            auto noise_floor_min = std::numeric_limits<long double>::infinity();
            for (size_t i = 0U; i < test_samples; ++i) {

                const auto sw_start = std::chrono::high_resolution_clock::now();

                noise_floor_min = std::min(
                    noise_floor_min,
                    std::chrono::duration_cast<std::chrono::duration<long double>>(
                        std::chrono::high_resolution_clock::now() - sw_start
                    ).count()
                );
            }

            /* TEST ALGORITHM */

            std::vector<coord_t> path;

            auto result = std::numeric_limits<long double>::infinity();
            for (size_t i = 0U; i < test_samples; ++i) {

                const auto sw_start = std::chrono::high_resolution_clock::now();

                struct params {

                    using weight_type = weight_t;

                    const decltype(maze)       _maze;
                    const coord_t              _start;
                    const coord_t              _end;
                    const decltype(&HEURISTIC) _h;
                    const scalar_t             _weight      =  1U;
                    const size_t               _capacity    =  0U;
                    const size_t               _memoryLimit = -1U;
                };

                auto solver = chdr::solvers::make_solver<chdr::solvers::astar, Kd, scalar_t, index_t, params>();
                path = solver(maze, start, end, HEURISTIC);

                result = std::min(
                    result,
                    std::chrono::duration_cast<std::chrono::duration<long double>>(
                        std::chrono::high_resolution_clock::now() - sw_start
                    ).count()
                );
            }

            if (drawable) {
                display<weight_t, Kd>::draw_maze(start, end, size, maze, path);
            }

            debug::log("(A*):");
            debug::log("\t" + std::string(path.size() != 0U ? "[SOLVED]" : "[IMPOSSIBLE]") +
                "\t(<= ~" + chdr::utils::to_string(std::max(result - noise_floor_min, std::numeric_limits<long double>::epsilon())) + ") / "
                  "(<= ~" + chdr::utils::to_string((result - noise_floor_min) / static_cast<long double>(path.size())) + "/n)");

            #undef HEURISTIC
        }
    };

} //test::tests

#endif //TEST_ASTAR_HPP