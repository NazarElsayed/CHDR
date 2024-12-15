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

    template<size_t Kd, typename scalar_t, typename index_t, typename params_t>
    struct [[maybe_unused]] floodfill final {

        static_assert(std::is_integral_v<index_t>, "index_t must be an integral type.");

    private:

        using   solver_t = solver<floodfill, Kd, scalar_t, index_t, params_t>;
        using open_set_t = queue<index_t>;

    public:

        [[maybe_unused, nodiscard]] static constexpr bool solve(const params_t& _params) {

            const auto s = utils::to_1d(_params.start, _params.size);
            const auto e = utils::to_1d(_params.end,   _params.size);

            if (_params.maze.contains(s)       &&
                _params.maze.contains(e)       &&
                _params.maze.at(s).is_active() &&
                _params.maze.at(e).is_active()
            ) {

                if (s != e) {

                    const auto capacity = std::max(_params.capacity, std::max(s, e));

                    // Create closed Set:
                    existence_set closed ({ s }, capacity);

                    // Create open set:
                    open_set_t open;
                    open.reserve(capacity / 8U);
                    open.emplace(s);

                    // Main loop:
                    while (!open.empty()) {

                        for (size_t i = 0U; i < open.size(); ++i) {

                            const auto curr(std::move(open.front()));
                            open.pop();

                            if (curr != e) {

                                closed.allocate(curr, _params.capacity, _params.maze.count());
                                closed.emplace(curr);

                                for (const auto& n_data : _params.maze.get_neighbours(curr)) {

                                    if (const auto n = solver_t::get_data(n_data, _params); n.active) {

                                        // Check if node is not already visited:
                                        if (!closed.contains(n.index)) {
                                             closed.allocate(n.index, capacity, _params.maze.count());
                                             closed.emplace(n.index);
                                               open.emplace(n.index); // Note: 'current' is now moved!
                                        }
                                    }
                                }
                            }
                            else {
                                return true;
                            }
                        }
                    }
                }
                else {
                    return true;
                }
            }

            return false;
        }
    };

} //chdr::solvers

#endif //CHDR_FLOODFILL_HPP