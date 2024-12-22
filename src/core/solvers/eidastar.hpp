/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_EIDASTAR_HPP
#define CHDR_EIDASTAR_HPP

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
    struct [[maybe_unused]] eidastar final {

        friend struct solver<eidastar, Kd, scalar_t, index_t, params_t>;

        static_assert(std::is_arithmetic_v<scalar_t>, "scalar_t must be an integral or floating point type.");
        static_assert(std::numeric_limits<scalar_t>::is_specialized, "scalar_t must be a numeric type with defined numeric limits.");
        static_assert(std::is_integral_v<index_t>, "index_t must be an integral type.");

    private:

        using solver_t = solver<eidastar, Kd, scalar_t, index_t, params_t>;
        using  coord_t = coord<index_t, Kd>;

        struct node final : bnode<index_t> {

            scalar_t m_gScore;
            scalar_t m_fScore;

            /**
             * @brief Constructs an uninitialized node.
             *
             * This constructor creates a node with uninitialized members.
             */
            // ReSharper disable once CppPossiblyUninitializedMember
            [[nodiscard]] constexpr node() noexcept : bnode<index_t>() {} // NOLINT(*-pro-type-member-init, *-use-equals-default)

            [[nodiscard]] constexpr node(const index_t& _index, const scalar_t& _gScore, const scalar_t& _fScore) noexcept : bnode<index_t>(_index),
                m_gScore(_gScore),
                m_fScore(_fScore) {}

            [[nodiscard]] friend constexpr bool operator < (const node& _a, const node& _b) noexcept {
                return _a.m_fScore == _b.m_fScore ?
                       _a.m_gScore >  _b.m_gScore :
                       _a.m_fScore >  _b.m_fScore;
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

        struct index_hash {
            constexpr size_t operator () (const index_t& _index) const noexcept { return static_cast<index_t>(_index); }
        };

        struct index_equal {
            constexpr bool operator () (const index_t& _a, const index_t& _b) const noexcept { return _a == _b; }
        };

        using transposition_table_t = std::unordered_map<index_t, scalar_t, index_hash, index_equal>;

        template <typename open_set_t>
        [[nodiscard]] static constexpr auto backtrack(const open_set_t& _open, const coord_t& _size, const size_t& _capacity = 1U) {

            // Reserve space in result:
            std::vector<coord_t> result;
            result.reserve(_capacity);

            // Recurse from end node to start node, inserting into a result buffer:
            for (const auto& t : _open) {
                result.emplace_back(utils::to_nd(t.m_index, _size));
            }

            // Reverse the result:
            std::reverse(result.begin(), result.end());

            return result;
        }

        template <typename open_set_t>
        [[nodiscard]] static constexpr auto solve_internal(open_set_t& _open, const params_t& _params) {

            using neighbours_t = decltype(_params.maze.get_neighbours());

            const auto s = utils::to_1d(_params.start, _params.size);
            const auto e = utils::to_1d(_params.end,   _params.size);

            auto min = std::numeric_limits<scalar_t>::max();

            auto bound = _params.h(_params.start, _params.end) * _params.weight;

            _open.emplace_back(s, static_cast<scalar_t>(0), bound);
            auto curr = _open.back();

            stack<state<neighbours_t>> stack;
            stack.emplace(curr, bound, _params);

            transposition_table_t transposition_table;
            transposition_table[curr.m_index] = bound;

            // Main loop:
            while (!stack.empty()) {

                auto& _ = stack.top();
                curr = std::move(_.curr);

                if (_.neighbours_idx != _.neighbours.size()) {
                    const auto& n_data = _.neighbours[_.neighbours_idx++];

                    if (const auto& n = solver_t::get_data(n_data, _params); n.active) {

                        auto g = curr.m_gScore + n.distance;
                        auto f = g + (_params.h(n.coord, _params.end) * _params.weight);

                        auto search = transposition_table.find(n.index);
                        if (!(search != transposition_table.end() && f >= search->second)) {
                            *transposition_table[n.index] = f;

                            _open.emplace_back(n.index, g, f);

                            if (n.index != e) { // SEARCH FOR SOLUTION...
                                stack.emplace(_open.back(), _.bound, _params);
                            }
                            else { // SOLUTION REACHED ...
                                return backtrack(_open, _params.size, curr.m_gScore);
                            }
                        }
                    }
                }
                else {
                    min = std::min(min, curr.m_fScore);

                    _open.pop_back();
                    stack.pop();
                }
            }

            return std::vector<coord_t>{};
        }

        [[maybe_unused, nodiscard]] static constexpr auto execute(const params_t& _params) {

            const auto capacity = solver_t::determine_capacity(_params);

            std::vector<node> open;
            open.reserve(capacity / 8U);

            return solve_internal(open, _params);
        }
    };

} //chdr::solvers

#endif //CHDR_EIDASTAR_HPP