/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_EIDDFS_HPP
#define CHDR_EIDDFS_HPP

#include <cstddef>
#include <limits>
#include <vector>

#include "base/solver.hpp"
#include "types/coord.hpp"
#include "types/stack.hpp"
#include "utils/intrinsics.hpp"
#include "utils/utils.hpp"

namespace chdr::solvers {

    template<typename params_t>
    struct [[maybe_unused]] eiddfs final {

        friend class solver<eiddfs, params_t>;

    private:

        using  index_t = typename params_t:: index_type;
        using  coord_t = typename params_t:: coord_type;
        using solver_t = solver<eiddfs, params_t>;

        static_assert(std::is_integral_v<index_t>, "index_t must be an integral type.");

        struct node final : bnode<index_t> {

            index_t m_depth;

            /**
             * @brief Constructs an uninitialized node.
             *
             * This constructor creates a node with uninitialized members.
             */
            // ReSharper disable once CppPossiblyUninitializedMember
            [[nodiscard]] constexpr node() noexcept : bnode<index_t>() {} // NOLINT(*-pro-type-member-init, *-use-equals-default)

            [[nodiscard]] constexpr node(const index_t& _index, const index_t& _depth) noexcept : bnode<index_t>(_index),
                m_depth(_depth) {}

            [[nodiscard]] friend constexpr bool operator < (const node& _a, const node& _b) noexcept {
                return _a.m_hScore > _b.m_hScore;
            }
        };

        template <typename neighbours_t>
        struct state {

            neighbours_t neighbours;
            index_t      neighbours_idx;

            state(const node& _curr, const params_t& _params) :
                neighbours(_params.maze.get_neighbours(_curr.m_index)),
                neighbours_idx(0U) {}
        };

        template <typename open_set_t>
        [[nodiscard]] static constexpr auto solve_internal(open_set_t& _open, const size_t& _capacity, const params_t& _params) {

            using neighbours_t = decltype(_params.maze.get_neighbours());

            const auto s = utils::to_1d(_params.start, _params.size);
            const auto e = utils::to_1d(_params.end,   _params.size);

            _open.emplace_back(s, 0U);

            stack<state<neighbours_t>> stack;

            existence_set transposition_table;

            for (size_t bound = 0U; bound < std::numeric_limits<size_t>::max(); ++bound) {

                stack.emplace(_open.back(), _params);

                // Main loop:
                while (!stack.empty()) {

                    auto& _ = stack.top();
                    auto& curr = _open.back();

                    if (curr.m_depth <= bound && _.neighbours_idx != _.neighbours.size()) {
                        const auto& n_data = _.neighbours[_.neighbours_idx++];

                        if (const auto& n = solver_t::get_data(n_data, _params); n.active) {

                            if (!transposition_table.contains(n.index)) {
                                utils::preallocate_emplace(transposition_table, n.index, _capacity, _params.maze.count());

                                _open.emplace_back(n.index, curr.m_depth + 1U);

                                if (n.index != e) { // SEARCH FOR SOLUTION...
                                    stack.emplace(_open.back(), _params);
                                }
                                else { // SOLUTION REACHED ...

                                    stack = {};

                                    const auto result = utils::ibacktrack(_open, _params.size);

                                    _open = {};

                                    return result;
                                }
                            }
                        }
                    }
                    else {
                        _open.pop_back();
                        stack.pop();
                        transposition_table.erase(curr.m_index);
                    }
                }

                _open.erase(_open.begin() + 1U, _open.end());
                stack.clear();
                transposition_table.clear();
            }

            _open = {};
            stack = {};

            return std::vector<coord_t>{};
        }

        [[maybe_unused, nodiscard]] static auto execute(const params_t& _params) {

            const auto capacity = solver_t::determine_capacity(_params);

            std::vector<node> open;
            try {
                open.reserve(capacity / 8U);
            }
            catch ([[maybe_unused]] const std::exception& e) {} // NOLINT(*-empty-catch)

            return solve_internal(open, capacity, _params);
        }
    };

} //chdr::solvers

#endif //CHDR_EIDDFS_HPP