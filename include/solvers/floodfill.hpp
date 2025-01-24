/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_FLOODFILL_HPP
#define CHDR_FLOODFILL_HPP

#include <cstddef>
#include <vector>

#include "../types/containers/existence_set.hpp"
#include "../types/containers/queue.hpp"
#include "../utils/utils.hpp"
#include "base/solver.hpp"

namespace chdr::solvers {

    template<typename params_t>
    struct [[maybe_unused]] floodfill final {

    private:

        using  index_t = typename params_t::index_type;
        using  coord_t = typename params_t::coord_type;
        using solver_t = solver<floodfill, params_t>;

        static_assert(std::is_integral_v<index_t>, "index_t must be an integral type.");

        template <typename open_set_t, typename closed_set_t>
        [[nodiscard]] HOT static constexpr bool solve_internal(open_set_t& _open, closed_set_t& _closed, size_t _capacity, const params_t& _params) {

            const auto s = utils::to_1d(_params.start, _params.size);
            const auto e = utils::to_1d(_params.end,   _params.size);

              _open.emplace(s);
            _closed.emplace(s);

            // Main loop:
            while (LIKELY(!_open.empty())) {

                for (size_t i = 0U; i < _open.size(); ++i) {

                    const auto curr(std::move(_open.front()));
                    _open.pop();

                    if (curr != e) { // SEARCH FOR SOLUTION...

                        for (const auto& n_data : _params.maze.get_neighbours(curr)) {

                            if (const auto& n = solver_t::get_data(n_data, _params); n.active) {

                                // Check if node is not already visited:
                                if (!_closed.contains(n.index)) {
                                    solver_t::solver_utils::preallocate_emplace(_closed, n.index, _capacity, _params.maze.count());

                                    _open.emplace(n.index);
                                }
                            }
                        }
                    }
                    else { // SOLUTION REACHED ...
                        return true;
                    }
                }
            }

            return false;
        }

    public:

        [[maybe_unused, nodiscard]] static auto invoke(const params_t& _params) {

            const auto s = static_cast<index_t>(utils::to_1d(_params.start, _params.size));
            const auto e = static_cast<index_t>(utils::to_1d(_params.end,   _params.size));

            if (_params.maze.contains(s) && _params.maze.at(s).is_active() &&
                _params.maze.contains(e) && _params.maze.at(e).is_active()
            ) {

                bool success { s == e };

                if (!success) {

                    const auto capacity = solver_t::solver_utils::determine_capacity(_params);

                    existence_set closed(_params.monotonic_pmr);
                    closed.reserve(capacity);

                    queue<index_t> open(_params.polytonic_pmr);

                    success = solve_internal(open, closed, capacity, _params);
                }

                if (success) {
                    return std::vector<coord_t> { _params.end };
                }
            }

            return std::vector<coord_t>{};
        }
    };

} //chdr::solvers

#endif //CHDR_FLOODFILL_HPP