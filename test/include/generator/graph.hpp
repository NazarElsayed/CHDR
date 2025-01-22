/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef TEST_GRAPH_HPP
#define TEST_GRAPH_HPP

#include <chdr.hpp>

#include <debug.hpp>

#include <algorithm>
#include <random>
#include <vector>

#include "utils/lcg.hpp"

namespace test::generator {

	struct graph final {

	    using rng_engine_t = utils::lcg<size_t>;

	    static constexpr size_t null_v = static_cast<size_t>(-1U);

        template <typename weight_t, typename index_t, typename coord_t, typename scalar_t>
        static auto generate(const coord_t& _start, coord_t& _end, const coord_t& _size, size_t _seed = null_v) {

			static_assert(std::is_integral_v<weight_t>, "Type weight_t must be an integral type.");

			constexpr auto Kd = std::tuple_size_v<std::decay_t<coord_t>>;
			
            constexpr bool bidirectional = true;

            _end = _start;

            debug::log("(Graph):");
            debug::log("\tRandom Spanning Tree\t (Seed " + std::to_string(_seed) + ")");

            chdr::mazes::graph<index_t, scalar_t> result;

            const auto seed = _seed == null_v ? std::random_device().operator()() : _seed;
            rng_engine_t rng(seed);

            const auto maxIndex = chdr::utils::product<index_t>(_size);

            std::vector<index_t> keys;
            std::unordered_map<index_t, size_t> depths;
            size_t maxDepth = 0U;

            {
                const auto s = chdr::utils::to_1d(_start, _size);
                result.add(s, {});
                keys.emplace_back(s);
                depths[s] = maxDepth;
            }

            size_t branchFactor = 0U;

            const auto [distanceMin, distanceMax] = std::make_pair(
                static_cast<scalar_t>(1),
                static_cast<scalar_t>(10)
            );

            index_t count{0};

            while (count + branchFactor < maxIndex) {

                // Get random node from graph:
                auto curr = keys[rng() % keys.size()];

                if (!depths.contains(curr)) {
                    depths[curr] = 0U;
                }
                auto& depth = depths[curr];

                if (depth > maxDepth) {
                    maxDepth = depth;

                    _end = chdr::utils::to_nd(curr, _size);
                }

                if (result.get_neighbours(curr).size() <= 1U) {

                    constexpr bool includeDiagonals = false;

                    size_t rand = (rng() % (includeDiagonals ? chdr::utils::powui(static_cast<size_t>(3U), Kd) - 1U : Kd * 2U));

                    branchFactor = chdr::utils::max(rand, static_cast<size_t>(2U));

                    // Add the branches:
                    for (size_t i = 1U, j = 0U; j < branchFactor; ++j, ++i) {

                        const auto next = count + i;

                        scalar_t distance{};
                        if constexpr (std::is_integral_v<scalar_t>) {
                            distance = std::uniform_int_distribution(distanceMin, distanceMax)(rng);
                        }
                        else {
                            distance = std::uniform_real_distribution(distanceMin, distanceMax)(rng);
                        }

                        result.add(curr, {next, distance});

                        if constexpr (bidirectional) {
                            result.add(next, {curr, distance});
                        }
                        else {
                            result.push(next);
                        }

                        keys.emplace_back(next);
                        depths[next] = depths[curr] + 1U;
                    }

                    count += branchFactor;
                }
            }

            debug::log("\t[FINISHED] \t(~" + chdr::utils::trim_trailing_zeros(std::to_string(count / static_cast<long double>(1000000000.0))) + "b total candidate nodes)");

			return result;
		}
	};

} //test::generator

#endif //TEST_GRAPH_HPP