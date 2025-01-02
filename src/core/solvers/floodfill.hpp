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

#include "base/solver.hpp"
#include "types/existence_set.hpp"
#include "types/queue.hpp"
#include "utils/utils.hpp"

namespace chdr::solvers {

    template<size_t Kd, typename scalar_t, typename index_t, typename params_t>
    struct [[maybe_unused]] floodfill final {

        static_assert(std::is_integral_v<index_t>, "index_t must be an integral type.");

    private:

        using solver_t = solver<floodfill, Kd, scalar_t, index_t, params_t>;
        using  coord_t = coord<index_t, Kd>;

        template <typename open_set_t, typename closed_set_t>
        [[nodiscard]] static constexpr auto solve_internal(open_set_t& _open, closed_set_t& _closed, const size_t& _capacity, const params_t& _params) {

            const auto s = utils::to_1d(_params.start, _params.size);
            const auto e = utils::to_1d(_params.end,   _params.size);

            _open.emplace_nosort(s);

            _closed.allocate(s, _capacity, _params.maze.count());
            _closed.emplace (s);

            // Main loop:
            while (!_open.empty()) {

                for (size_t i = 0U; i < _open.size(); ++i) {

                    bool dirty = false;

                    const auto curr(std::move(_open.front()));
                    _open.pop();

                    if (curr != e) { // SEARCH FOR SOLUTION...

                        for (const auto& n_data : _params.maze.get_neighbours(curr)) {

                            if (const auto& n = solver_t::get_data(n_data, _params); n.active) {

                                // Check if node is not already visited:
                                if (!_closed.contains(n.index)) {
                                     _closed.allocate(n.index, _capacity, _params.maze.count());
                                     _closed.emplace (n.index);

                                    _open.emplace_nosort(n.index);

                                    dirty = true;
                                }
                            }
                        }

                        if (dirty) {
                            _open.reheapify(_open.back());
                        }
                    }
                    else { // SOLUTION REACHED ...
                        return true;
                    }
                }
            }
        }

    public:

        [[maybe_unused, nodiscard]] static constexpr std::vector<coord_t> execute(const params_t& _params) {

            const auto s = utils::to_1d(_params.start, _params.size);
            const auto e = utils::to_1d(_params.end,   _params.size);

            if (_params.maze.contains(s)       &&
                _params.maze.contains(e)       &&
                _params.maze.at(s).is_active() &&
                _params.maze.at(e).is_active()
            ) {

                bool success { s == e };

                if (!success) {

                    const auto capacity = solver_t::determine_capacity(_params);

                    existence_set closed ({ s }, capacity);

                    queue<index_t> open;
                    open.reserve(capacity / 8U);

                    success = solve_internal(closed, open);
                }

                if (success) {
                    return { _params.end() };
                }
            }

            return {};
        }
    };

} //chdr::solvers

#endif //CHDR_FLOODFILL_HPP