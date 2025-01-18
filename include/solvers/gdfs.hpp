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

#include "../types/containers/existence_set.hpp"
#include "../types/containers/stack.hpp"
#include "../utils/utils.hpp"
#include "base/managed_node.hpp"
#include "base/solver.hpp"

namespace chdr::solvers {

    template<typename params_t>
    struct [[maybe_unused]] gdfs final {

        friend class solver<gdfs, params_t>;

    private:

        using  index_t = typename params_t::index_type;
        using  coord_t = typename params_t::coord_type;
        using solver_t = solver<gdfs, params_t>;
        using     node = managed_node<index_t>;

        static_assert(std::is_integral_v<index_t>, "index_t must be an integral type.");

        template <typename open_set_t, typename closed_set_t>
        [[nodiscard]] static constexpr auto solve_internal(open_set_t& _open, closed_set_t& _closed, const size_t& _capacity, const params_t& _params) {

            const auto s = utils::to_1d(_params.start, _params.size);
            const auto e = utils::to_1d(_params.end,   _params.size);

              _open.emplace(s);
            _closed.emplace(s);

            // Main loop:
            while (!_open.empty()) { // SEARCH FOR SOLUTION...

                auto curr(std::move(_open.front()));
                _open.pop();

                if (curr.m_index != e) { // SEARCH FOR SOLUTION...

                    node* RESTRICT curr_ptr(nullptr);

                    for (const auto& n_data: _params.maze.get_neighbours(curr.m_index)) {

                        if (const auto& n = solver_t::get_data(n_data, _params); n.active) {

                            // Check if node is not already visited:
                            if (!_closed.contains(n.index)) {
                                solver_utils::preallocate_emplace(_closed, n.index, _capacity, _params.maze.count());

                                if (curr_ptr == nullptr) {
                                    curr_ptr = new (_params.pool_pmr->allocate(sizeof(node), alignof(node))) node(std::move(curr));
                                }

                                _open.emplace(n.index, curr_ptr);
                            }
                        }
                    }

                    if (curr_ptr == nullptr) {
                        curr.expunge(_params.pool_pmr);
                    }
                }
                else { // SOLUTION REACHED ...
                    return solver_utils::rbacktrack(curr, _params.size);
                }
            }

            return std::vector<coord_t>{};
        }

        [[maybe_unused, nodiscard]] static auto execute(const params_t& _params) {

            const auto capacity = solver_t::determine_capacity(_params);

            existence_set closed(_params.monotonic_pmr);
            closed.reserve(capacity);

            stack<node> open(_params.polytonic_pmr);

            return solve_internal(open, closed, capacity, _params);
        }
    };

} //chdr::solvers

#endif //CHDR_GDFS_HPP