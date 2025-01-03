/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_GDFS_HPP
#define CHDR_GDFS_HPP

#include <cstddef>
#include <vector>

#include "base/solver.hpp"
#include "solvers/base/managed_node.hpp"
#include "types/coord.hpp"
#include "types/existence_set.hpp"
#include "types/stack.hpp"
#include "utils/intrinsics.hpp"
#include "utils/utils.hpp"

namespace chdr::solvers {

    template<size_t Kd, typename scalar_t, typename index_t, typename params_t>
    struct [[maybe_unused]] gdfs final {

        friend class solver<gdfs, Kd, scalar_t, index_t, params_t>;

        static_assert(std::is_integral_v<index_t>, "index_t must be an integral type.");

    private:

        using   solver_t = solver<gdfs, Kd, scalar_t, index_t, params_t>;
        using    coord_t = coord<index_t, Kd>;
        using       node = managed_node<index_t>;

        template <typename open_set_t, typename closed_set_t>
        [[nodiscard]] static constexpr auto solve_internal(open_set_t& _open, closed_set_t& _closed, const size_t& _capacity, const params_t& _params) {

            std::vector<coord_t> result;

            const auto s = utils::to_1d(_params.start, _params.size);
            const auto e = utils::to_1d(_params.end,   _params.size);

            _open.emplace(s);

            _closed.allocate(s, _capacity, _params.maze.count());
            _closed.emplace (s);

            // Main loop:
            while (!_open.empty()) { // SEARCH FOR SOLUTION...

                auto curr(std::move(_open.front()));
                _open.pop();

                if (curr.m_index != e) { // SEARCH FOR SOLUTION...

                    node* RESTRICT curr_ptr = nullptr;

                    for (const auto& n_data: _params.maze.get_neighbours(curr.m_index)) {

                        if (const auto& n = solver_t::get_data(n_data, _params); n.active) {

                            // Check if node is not already visited:
                            if (!_closed.contains(n.index)) {
                                 _closed.allocate(n.index, _capacity, _params.maze.count());
                                 _closed.emplace (n.index);

                                if (curr_ptr == nullptr) {
                                    node::alloc.construct(curr_ptr = node::alloc.allocate(1U), std::move(curr)); // Note: 'current' is now moved!
                                }

                                _open.emplace(n.index, curr_ptr);
                            }
                        }
                    }

                    if (curr_ptr == nullptr) {
                        curr.expunge();
                    }
                }
                else { // SOLUTION REACHED ...

                    // Release:
                    _open = {};

                    result = curr.template backtrack<node>(_params.size, _capacity);
                    break;
                }
            }

            // Release:
            _closed = {};
            node::alloc.reset();

            return result;
        }

        [[maybe_unused, nodiscard]] static constexpr auto execute(const params_t& _params) {

            const auto s = utils::to_1d(_params.start, _params.size);

            const auto capacity = solver_t::determine_capacity(_params);

            existence_set closed({ s }, capacity);

            stack<node> open;

            return solve_internal(open, closed, capacity, _params);
        }
    };

} //chdr::solvers

#endif //CHDR_GDFS_HPP