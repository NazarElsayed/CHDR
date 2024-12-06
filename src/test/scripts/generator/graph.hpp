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

#include "Debug.hpp"

namespace test::generator {

	struct graph final {

        using uniform_rng_t = std::mt19937_64;

        template <typename T, typename index_t, typename scalar_t, const size_t Kd, typename... Args>
        static auto generate(const chdr::coord_t<size_t, Kd>& _start, chdr::coord_t<size_t, Kd>& _end, const float& _loops = 0.0F, const float& _obstacles = 0.0f, const size_t& _seed = -1U, const Args&... _size) {

			static_assert(std::is_integral_v<T>, "Type T must be an integral type.");

            constexpr bool bidirectional = true;

            _end = _start;

            Debug::Log("(Graph):");
            Debug::Log("\tRandom Spanning Tree\t (Seed " + std::to_string(_seed) + ")");

            std::array size { _size... };

            chdr::mazes::graph<index_t, scalar_t> result;

            constexpr size_t null_v = -1U;

            const auto seed = _seed == null_v ? std::random_device().operator()() : _seed;
            uniform_rng_t rng(seed);

            const auto max_index = chdr::Utils::Product<index_t>(size);

            std::vector<index_t> keys;
            std::unordered_map<index_t, size_t> depths;
            size_t max_depth = 0U;

            {
                const auto s = chdr::Utils::To1D(_start, size);
                result.add(s, {});
                keys.emplace_back(s);
                depths[s] = max_depth;
            }

            size_t branch_factor = 0U;

            const auto [distance_min, distance_max] = std::make_pair(
                static_cast<scalar_t>(1),
                static_cast<scalar_t>(10)
            );

            index_t count{0};

            while (count + branch_factor < max_index) {

                // Get random node from graph:
                auto curr = keys[rng() % keys.size()];

                if (depths.find(curr) == depths.end()) {
                    depths[curr] = 0U;
                }
                auto& depth = depths[curr];

                if (depth > max_depth) {
                    max_depth = depth;

                    _end = chdr::Utils::ToND(curr, size);
                }

                if (result.GetNeighbours(curr).size() <= 1U)  {

                    constexpr bool IncludeDiagonals = false;

                    size_t rand = (rng() % (IncludeDiagonals ?
                        static_cast<size_t>(std::pow(3U, Kd)) - 1U : Kd * 2U));

                    branch_factor = std::max(rand, static_cast<size_t>(2UL));

                    // Add the branches:
                    for (size_t i = 1U, j = 0U; j < branch_factor; ++j, ++i) {

                        const auto next = count + i;

                        scalar_t distance{};
                        if constexpr (std::is_integral_v<scalar_t>) {
                            distance = std::uniform_int_distribution(distance_min, distance_max)(rng);
                        }
                        else {
                            distance = std::uniform_real_distribution(distance_min, distance_max)(rng);
                        }

                        result.add(curr, {next, distance});

                        if constexpr (bidirectional) {
                            result.add(next, {curr, distance});
                        }
                        else {
                            result.add(next);
                        }

                        keys.emplace_back(next);
                        depths[next] = depths[curr] + 1U;
                    }

                    count += branch_factor;
                }
            }

            Debug::Log("\t[FINISHED] \t(~" + chdr::Utils::Trim_Trailing_Zeros(std::to_string(count / static_cast<long double>(1000000000.0))) + "b total candidate nodes)");

			return result;
		}
	};
}

#endif //TEST_GRAPH_HPP