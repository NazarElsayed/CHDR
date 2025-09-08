/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef TEST_OBSTACLES_HPP
#define TEST_OBSTACLES_HPP

#include <chdr.hpp>
#include <debug.hpp>

#include <cstddef>

#include "utils/lcg.hpp"

namespace test::generator {

    struct obstacles final {

    private:

        static constexpr size_t null_v = static_cast<size_t>(-1U);

    public:

        template <typename weight_t, typename index_t, typename coord_t, typename scalar_t>
        static
#if __cplusplus >= 202300L
        constexpr
#endif // __cplusplus >= 202300L
        auto generate(const coord_t& _start, coord_t& _end, const coord_t& _size, double _obstacles, size_t _iterations = std::numeric_limits<size_t>::max(), size_t _seed = null_v) {

            static_assert(std::is_integral_v<weight_t>, "Type T must be an integral type.");

            using rng_engine_t = utils::lcg<size_t>;

            auto nodes = std::vector<weight_t>(chdr::utils::product<size_t>(_size));

            debug::log("(Maze):");

            // (Ignore 0D & 1D mazes.)
            if (_size.size() >= 2U && _size[0U] > 2U && _size[1U] > 2U) {

                // Generate random 2D->KD mazes:
                for (size_t i = 0U; i < _iterations; ++i) {

                    // Seed the random number generator.
                    const auto seed = _seed == null_v ? static_cast<size_t>(time(nullptr)) : _seed;
                    rng_engine_t rng(seed + i);

                    const auto threshold = static_cast<size_t>(static_cast<decltype(_obstacles)>(std::numeric_limits<size_t>::max()) * _obstacles);

                    // Create a random maze using the given threshold for obstacles:
                    for (size_t j = 1U; j < nodes.size() - 1UL; ++j) {
                        nodes[j] = rng.operator()() <= threshold ?
                            std::numeric_limits<weight_t>::max() :
                            std::numeric_limits<weight_t>::lowest();
                    }

                    // Check if maze is solvable:
                    if (_iterations != 0U) {

                        auto monotonic     = chdr::monotonic_pool();
                        auto heterogeneous = chdr::heterogeneous_pool();
                        auto homogeneous   = chdr::homogeneous_pool();

                        struct params final {

                            using  weight_type [[maybe_unused]] = weight_t;
                            using  scalar_type [[maybe_unused]] = scalar_t;
                            using   index_type [[maybe_unused]] =  index_t;
                            using   coord_type [[maybe_unused]] =  coord_t;

                            using        lazy_sorting [[maybe_unused]] = std::true_type;
                            using          no_cleanup [[maybe_unused]] = std::false_type;
                            using reverse_equivalence [[maybe_unused]] = std::true_type;

                            const chdr::mazes::grid<coord_t, weight_t>& maze;
                            const coord_type start;
                            const coord_type end;
                            const coord_type size;
                            scalar_type (*h)(const coord_type&, const coord_type&) noexcept;

                            decltype(    monotonic)* monotonic_pmr;
                            decltype(heterogeneous)* heterogeneous_pmr;
                            decltype(  homogeneous)* homogeneous_pmr;

                            const scalar_type weight      = 1U;
                            const      size_t capacity    = 0U;
                            const      size_t memoryLimit = static_cast<size_t>(-1U);
                        };

                        auto path = chdr::solvers::solver<chdr::solvers::gbest_first, params>::solve({ { _size, nodes }, _start, _end, _size, chdr::heuristics::manhattan_distance<scalar_t, coord_t>, &monotonic, &heterogeneous, &homogeneous });
                        if (path.empty()) {
                            if (i == _iterations - 1U) {
                                throw std::runtime_error("ERROR: Could not create a solvable maze!");
                            }
                        }
                        else {
                            std::cout << "Solution depth (d) = " << path.size() << "\n";
                            break;
                        }
                    }
                }
            }

            debug::log("\t[FINISHED] \t(~" + chdr::utils::trim_trailing_zeros(std::to_string(chdr::utils::product<size_t>(_size) / 1000000000.0L)) + "b total candidate nodes)");

            return chdr::mazes::grid<coord_t, weight_t> { _size, nodes };
        }
    };

} //test::generator

#endif //TEST_OBSTACLES_HPP