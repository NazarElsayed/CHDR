/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef TEST_SOLVER_HPP
#define TEST_SOLVER_HPP

#include <debug.hpp>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <type_traits>
#include <vector>

#include "core/display.hpp"

// ReSharper disable CppUnusedIncludeDirective
#include "generator/graph.hpp"   // NOLINT(*-include-cleaner)
#include "generator/grid.hpp"    // NOLINT(*-include-cleaner)
// ReSharper restore CppUnusedIncludeDirective

namespace test {

    struct solver final {

        template <template <typename params_t> typename solver_t, typename params_t, typename... Args>
        static void run(Args&&... _args) {

            constexpr auto Kd = std::tuple_size_v<std::decay_t<typename params_t::coord_type>>;

            const params_t _params { std::forward<Args>(_args)... };

            /* TEST SAMPLES */
#ifndef NDEBUG
            constexpr size_t base_samples = 1000000UL;
#else //!NDEBUG
            constexpr size_t base_samples = 100000000UL;
#endif //!NDEBUG
            const size_t test_samples = chdr::utils::max(base_samples / _params.maze.count(), static_cast<size_t>(1U));

            std::this_thread::sleep_for(std::chrono::milliseconds(10U));

            /* CAPTURE SYSTEM NOISE */
            auto noise_floor_min = std::numeric_limits<long double>::max();
            for (size_t i = 0U; i < test_samples; ++i) {

                const auto sw_start = std::chrono::high_resolution_clock::now();

                noise_floor_min = chdr::utils::min(
                    noise_floor_min,
                    std::chrono::duration_cast<std::chrono::duration<long double>>(
                        std::chrono::high_resolution_clock::now() - sw_start
                    ).count()
                );
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10U));

            /* TEST ALGORITHM */
            debug::log("(Solver):");
            std::vector<typename params_t::coord_type> path;

            auto result = std::numeric_limits<long double>::max();
            for (size_t i = 0U; i < test_samples; ++i) {

                const auto sw_start = std::chrono::high_resolution_clock::now();

                /* EXECUTE SOLVE */
                {
                    path = chdr::solvers::solve<solver_t, params_t>(_params);
                }

                result = chdr::utils::min(
                    result,
                    std::chrono::duration_cast<std::chrono::duration<long double>>(
                        std::chrono::high_resolution_clock::now() - sw_start
                    ).count()
                );
            }

            if (std::is_base_of_v<chdr::mazes::grid<typename params_t::coord_type, typename params_t::weight_type>, std::decay_t<decltype(_params.maze)>> &&
                _params.maze.size()[0U] <= 100U &&
                _params.maze.size()[1U] <= 100U &&
                Kd > 0U &&
                Kd < 3U
            ) {
                display::draw_maze(_params.start, _params.end, _params.maze, path);
            }

            debug::log("\t" + std::string(!path.empty() ? "[SOLVED]" : "[IMPOSSIBLE]") +
                "\t(<= ~" + chdr::utils::to_string(chdr::utils::max(result - noise_floor_min, std::numeric_limits<long double>::epsilon())) + ") / "
                  "(<= ~" + chdr::utils::to_string((result - noise_floor_min) / static_cast<long double>(path.size() > 1U ? path.size() + 1U : path.size()), 0.2l) + "/n)");
        }
    };

} //test::solver

#endif //TEST_SOLVER_HPP