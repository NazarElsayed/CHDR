/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_DFS_HPP
#define CHDR_DFS_HPP

#include <cstddef>
#include <vector>

#include "base/solver.hpp"
#include "base/unmanaged_node.hpp"
#include "types/coord.hpp"
#include "types/existence_set.hpp"
#include "types/append_only_allocator.hpp"
#include "types/stack.hpp"
#include "utils/intrinsics.hpp"
#include "utils/utils.hpp"

namespace chdr::solvers {

    template<size_t Kd, typename scalar_t, typename index_t, typename params_t>
    struct [[maybe_unused]] dfs final {

        friend class solver<dfs, Kd, scalar_t, index_t, params_t>;

        static_assert(std::is_integral_v<index_t>, "index_t must be an integral type.");

    private:

        using solver_t = solver<dfs, Kd, scalar_t, index_t, params_t>;
        using  coord_t = coord<index_t, Kd>;
        using     node = unmanaged_node<index_t>;

        template <typename open_set_t, typename closed_set_t, typename alloc_t>
        [[nodiscard]] static constexpr auto solve_internal(open_set_t& _open, closed_set_t& _closed, alloc_t& _alloc, const size_t& _capacity, const params_t& _params) {

            const auto s = utils::to_1d(_params.start, _params.size);
            const auto e = utils::to_1d(_params.end,   _params.size);

               _open.emplace(s);
             _closed.emplace(s);

            // Main loop:
            while (!_open.empty()) {

                auto curr(std::move(_open.top()));
                _open.pop();

                if (curr.m_index != e) { // SEARCH FOR SOLUTION...

                    node* RESTRICT curr_ptr = nullptr;

                    for (const auto& n_data : _params.maze.get_neighbours(curr.m_index)) {

                        if (const auto& n = solver_t::get_data(n_data, _params); n.active) {

                            // Check if node is not already visited:
                            if (!_closed.contains(n.index)) {
                                utils::preallocate_emplace(_closed, n.index, _capacity, _params.maze.count());

                                if (curr_ptr == nullptr) {
                                    _alloc.construct(curr_ptr = _alloc.allocate(1U), std::move(curr)); // Note: 'current' is now moved!
                                }

                                _open.emplace(n.index, curr_ptr);
                            }
                        }
                    }
                }
                else { // SOLUTION REACHED ...

                    _open   = {};
                    _closed = {};

                    return utils::rbacktrack(curr, _params.size);
                }
            }

            _open   = {};
            _closed = {};

            return std::vector<coord_t>{};
        }

        [[maybe_unused, nodiscard]] static auto execute(const params_t& _params) {

            const auto capacity = solver_t::determine_capacity(_params);

            existence_set<low_memory_usage> closed;
            closed.reserve(capacity);

            stack<node> open;

            append_only_allocator<node> alloc;

            return solve_internal(open, closed, alloc, capacity, _params);
        }
    };

} //chdr::solvers

#endif //CHDR_DFS_HPP