/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_FLOODFILL_HPP
#define CHDR_FLOODFILL_HPP

#include "mazes/graph.hpp"
#include "mazes/grid.hpp"
#include "mazes/base/imaze.hpp"
#include "types/existence_set.hpp"
#include "utils/utils.hpp"

namespace chdr::solvers {

    template<typename weight_t, const size_t Kd, typename scalar_t, typename index_t, typename params_t>
    class [[maybe_unused]] floodfill final {

        static_assert(std::is_integral_v<index_t>, "index_t must be an integral type.");

    public:

        [[maybe_unused]]
        auto solve(const params_t& _params) {

	        bool result = false;

            const auto s = utils::to_1d(_params._start, _params._maze.size());
            const auto e = utils::to_1d(_params._end,   _params._maze.size());

            if (_params._maze.contains(s) &&
                _params._maze.contains(e) &&
                _params._maze.at(s).is_active() &&
                _params._maze.at(e).is_active()
            ) {

                if (s != e) {

                    // Create closed Set:
                    existence_set closed ({ s }, std::max(_params._capacity, std::max(s, e)));

                    // Create open set:
                    queue<index_t> open;
                    open.emplace(s);

                    // Main loop:
                    while (!open.empty() && !result) {

                        for (size_t i = 0U; i < open.size(); ++i) {

                            const auto curr(std::move(open.front()));
                            open.pop();

                            if (curr != e) {

                                closed.allocate(curr, _params._capacity, _params._maze.count());
                                closed.emplace(curr);

                                for (const auto& neighbour : _params._maze.get_neighbours(curr)) {

                                    if constexpr (std::is_same_v<std::decay_t<decltype(_params._maze)>, mazes::graph<index_t, scalar_t>>) {

                                        const auto& n = neighbour.first;

                                        // Check if node is not already visited:
                                        if (!closed.contains(n)) {
                                             closed.allocate(n, _params._capacity, _params._maze.count());
                                             closed.emplace(n);
                                               open.emplace(n);
                                        }
                                    }
                                    else {

                                        if (const auto& [nActive, nCoord] = neighbour; nActive) {

                                            const auto n = utils::to_1d(nCoord, _params._maze.size());

                                            // Check if node is not already visited:
                                            if (!closed.contains(n)) {
                                                 closed.allocate(n, _params._capacity, _params._maze.count());
                                                 closed.emplace(n);
                                                   open.emplace(n);
                                            }
                                        }
                                    }
                                }
                            }
                            else {
                                result = true;

                                break;
                            }
                        }
                    }
                }
                else {
                    result = true;
                }
            }

            return result;
        }
    };

} //chdr::solvers

#endif //CHDR_FLOODFILL_HPP