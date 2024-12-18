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
#ifndef NDEBUG
            constexpr size_t base_samples = 1000000UL;
#else //!NDEBUG
            constexpr size_t base_samples = 100000000UL;
#endif //!NDEBUG

            const size_t test_samples = std::max(base_samples / chdr::utils::product<size_t>(_dimensions), 1UL);

            constexpr size_t seed(0U);

            const     coord_t size = _dimensions;
            constexpr coord_t start {};
                      coord_t end;

            /* MAX DRAW SIZE */
            bool drawable (
                size[0U] <= 100U &&
                size[1U] <= 100U &&
                      Kd >    0U &&
                      Kd <    3U
            );

            /* GENERATE MAZE */

            const auto grid = generator::grid::generate<weight_t>(start, end, 0.0, 0.0, seed, size);

            const auto test = grid;
            //const auto test = chdr::mazes::graph<index_t, scalar_t>(grid);
            //const auto test = generator::graph::generate<weight_t, index_t, scalar_t>(start, end, 0.0, 0.0, seed, size); drawable = false;

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

                    using weight_type [[maybe_unused]] = weight_t;

                    const decltype(test)       maze;
                    const coord_t              start;
                    const coord_t              end;
                    const coord_t              size;
                    const decltype(&HEURISTIC) h;
                    const scalar_t             weight      =  1U;
                    const size_t               capacity    =  0U;
                    const size_t               memoryLimit = -1U;
                };

                const auto solver = chdr::solvers::make_solver<chdr::solvers::gstar, Kd, scalar_t, index_t, params>();
                path = solver(test, start, end, size, HEURISTIC);

                result = std::min(
                    result,
                    std::chrono::duration_cast<std::chrono::duration<long double>>(
                        std::chrono::high_resolution_clock::now() - sw_start
                    ).count()
                );
            }

            if (drawable) {
                display<weight_t, Kd>::draw_maze(start, end, size, grid, path);
            }

            debug::log("(A*):");
            debug::log("\t" + std::string(!path.empty() ? "[SOLVED]" : "[IMPOSSIBLE]") +
                "\t(<= ~" + chdr::utils::to_string(std::max(result - noise_floor_min, std::numeric_limits<long double>::epsilon())) + ") / "
                  "(<= ~" + chdr::utils::to_string((result - noise_floor_min) / static_cast<long double>(path.size() > 1U ? path.size() + 1U : path.size())) + "/n)");

            #undef HEURISTIC
        }
    };

} //test::tests

#endif //TEST_ASTAR_HPP