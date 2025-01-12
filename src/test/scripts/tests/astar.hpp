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

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
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

        template <typename weight_t, typename coord_t, typename scalar_t = uint32_t, typename index_t = uint32_t>
        static void run(const coord_t& _dimensions) {

            constexpr auto Kd = std::tuple_size_v<std::decay_t<coord_t>>;

            /* TEST SAMPLES */
#ifndef NDEBUG
            constexpr size_t base_samples = 1000000UL;
#else //!NDEBUG
            constexpr size_t base_samples = 100000000UL;
#endif //!NDEBUG
            const size_t test_samples = std::max(base_samples / chdr::utils::product<size_t>(_dimensions), static_cast<size_t>(1U));

            const     coord_t size = _dimensions;
            constexpr coord_t start {};
                      coord_t end;

            /* GENERATE MAZE */
            constexpr auto seed { 0U };
            const auto grid = generator::grid::generate<weight_t>(start, end, size, 0.0, 0.0, seed);

            const auto test = grid;
            //const auto test = chdr::mazes::graph<index_t, scalar_t>(grid);
            //const auto test = generator::graph::generate<weight_t, index_t, coord_t, scalar_t>(start, end, size, seed);

            /* CAPTURE SYSTEM NOISE */
            auto noise_floor_min = std::numeric_limits<long double>::max();
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
            debug::log("(A*):");
            std::vector<coord_t> path;

            auto result = std::numeric_limits<long double>::max();
            for (size_t i = 0U; i < test_samples; ++i) {

                const auto sw_start = std::chrono::high_resolution_clock::now();

                {
                    struct params {

                        using weight_type [[maybe_unused]] = weight_t;
                        using scalar_type                  = scalar_t;
                        using  index_type                  =  index_t;
                        using  coord_type                  =  coord_t;

                        const decltype(test) maze;
                        const  coord_type    start;
                        const  coord_type    end;
                        const  coord_type    size;
                              scalar_type    (*h)(const coord_type&, const coord_type&) noexcept;
                        const scalar_type    weight      =  1U;
                        const  index_type    capacity    =  0U;
                        const  index_type    memoryLimit = -1U;
                    };

                    const auto solver = chdr::solvers::make_solver<chdr::solvers::astar, params>();
                    path = solver(test, start, end, size, chdr::heuristics::manhattan_distance<scalar_t, coord_t>);
                }

                result = std::min(
                    result,
                    std::chrono::duration_cast<std::chrono::duration<long double>>(
                        std::chrono::high_resolution_clock::now() - sw_start
                    ).count()
                );
            }

            if (std::is_same_v<std::decay_t<decltype(test)>, chdr::mazes::grid<coord_t, weight_t>> &&
                size[0U] <= 100U &&
                size[1U] <= 100U &&
                      Kd >    0U &&
                      Kd <    3U
            ) {
                display::draw_maze(start, end, size, grid, path);
            }

            debug::log("\t" + std::string(!path.empty() ? "[SOLVED]" : "[IMPOSSIBLE]") +
                "\t(<= ~" + chdr::utils::to_string(std::max(result - noise_floor_min, std::numeric_limits<long double>::epsilon())) + ") / "
                  "(<= ~" + chdr::utils::to_string((result - noise_floor_min) / static_cast<long double>(path.size() > 1U ? path.size() + 1U : path.size())) + "/n)");
        }
    };

} //test::tests

#endif //TEST_ASTAR_HPP