/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_IDBSTAR_HPP
#define CHDR_IDBSTAR_HPP

#include <cstddef>
#include <limits>
#include <vector>

#include "base/solver.hpp"
#include "types/coord.hpp"
#include "types/stack.hpp"
#include "utils/intrinsics.hpp"
#include "utils/utils.hpp"

namespace chdr::solvers {

    template<size_t Kd, typename scalar_t, typename index_t, typename params_t>
    struct [[maybe_unused]] idbstar final {

        friend class solver<idbstar, Kd, scalar_t, index_t, params_t>;

        static_assert(std::is_arithmetic_v<scalar_t>, "scalar_t must be an integral or floating point type.");
        static_assert(std::numeric_limits<scalar_t>::is_specialized, "scalar_t must be a numeric type with defined numeric limits.");
        static_assert(std::is_integral_v<index_t>, "index_t must be an integral type.");

    private:

        using solver_t = solver<idbstar, Kd, scalar_t, index_t, params_t>;
        using  coord_t = coord<index_t, Kd>;

        struct node final : bnode<index_t> {

            scalar_t m_hScore;

            /**
             * @brief Constructs an uninitialized node.
             *
             * This constructor creates a node with uninitialized members.
             */
            // ReSharper disable once CppPossiblyUninitializedMember
            [[nodiscard]] constexpr node() noexcept : bnode<index_t>() {} // NOLINT(*-pro-type-member-init, *-use-equals-default)

            [[nodiscard]] constexpr node(const index_t& _index, const scalar_t& _hScore) noexcept : bnode<index_t>(_index),
                m_hScore(_hScore) {}

            [[nodiscard]] friend constexpr bool operator < (const node& _a, const node& _b) noexcept {
                return _a.m_hScore > _b.m_hScore;
            }
        };

        template <typename neighbours_t>
        struct state {

            node curr;
            const scalar_t bound;

            neighbours_t neighbours;
            size_t       neighbours_idx;

            state(const node& _curr, const scalar_t& _bound, const params_t& _params) :
                curr(_curr),
                bound(_bound),
                neighbours(_params.maze.get_neighbours(curr.m_index)),
                neighbours_idx(0U) {}
        };

        template <typename open_set_t>
        [[nodiscard]] static constexpr auto backtrack(const open_set_t& _open, const coord_t& _size) {

            // Reserve space in result:
            std::vector<coord_t> result;
            result.reserve(_open.size());

            // Recurse from end node to start node, inserting into a result buffer:
            for (auto it = _open.rbegin(); it != _open.rend(); ++it) {
                result.emplace_back(utils::to_nd(it->m_index, _size));
            }

            return result;
        }

        template <typename open_set_t>
        [[nodiscard]] static constexpr auto solve_internal(open_set_t& _open, const size_t& _capacity, const params_t& _params) {

            using neighbours_t = decltype(_params.maze.get_neighbours());

            const auto s = utils::to_1d(_params.start, _params.size);
            const auto e = utils::to_1d(_params.end,   _params.size);

            auto min = std::numeric_limits<scalar_t>::max();

            auto bound = _params.h(_params.start, _params.end) * _params.weight;

            _open.emplace_back(s, bound);

            stack<state<neighbours_t>> stack;
            stack.emplace(_open.back(), bound, _params);

            // Main loop:
            while (!stack.empty()) {

                auto& _ = stack.top();
                auto& curr = _.curr;

                if (_.neighbours_idx != _.neighbours.size()) {
                    const auto& n_data = _.neighbours[_.neighbours_idx++];

                    if (const auto& n = solver_t::get_data(n_data, _params); n.active) {

                        if (std::none_of(_open.begin(), _open.end(), [&n](const auto& _item) ALWAYS_INLINE { return _item.m_index == n.index; })) {

                            _open.emplace_back(n.index, _params.h(n.coord, _params.end) * _params.weight);

                            if (n.index != e) { // SEARCH FOR SOLUTION...
                                stack.emplace(_open.back(), _.bound, _params);
                            }
                            else { // SOLUTION REACHED ...

                                stack = {};

                                const auto result = backtrack(_open, _params.size);

                                _open = {};

                                return result;
                            }
                        }
                    }
                }
                else {
                    min = std::min(min, curr.m_hScore);

                    _open.pop_back();
                    stack.pop();
                }
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

#endif //CHDR_IDBSTAR_HPP