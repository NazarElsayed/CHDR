/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_EID_BEST_FIRST_HPP
#define CHDR_EID_BEST_FIRST_HPP

/**
 * @file eidbest_first.hpp
 */

#include <limits>
#include <vector>

#include "../types/containers/stack.hpp"
#include "../utils/utils.hpp"
#include "base/bnode.hpp"
#include "base/solver.hpp"

// ReSharper disable once CppUnusedIncludeDirective
#include "../utils/intrinsics.hpp" // NOLINT(*-include-cleaner)

namespace chdr::solvers {

    /**
     * @struct eidbest_first
     * @brief Enhanced Iterative-deepening A* search algorithm.
     * @details Enhanced iterative-deepening best-first search is a best-first variant of the IDA*+ algorithm
     *          (Reinefeld, A. and Marsland, T. A., 1994).\n
     *
     * Advantages:
     * - Heuristic-driven search can improve search times when compared to IDDFS+.
     * - Lower memory usage when compared to standard best-first search.
     * - Improved performance by maintaining a record of the search state.
     * - Does not need a pre-pass, although performance can improve if the search space is pruned first.
     *
     * Limitations:
     * - Does not guarantee an optimal path according to the cost heuristic.
     * - Low performance due to repeated traversal of the search space.
     * - Slightly higher memory usage than iterative-deepening best-first search.
     *
     * References:
     * - Reinefeld, A. and Marsland, T. A., 1994. Enhanced iterative-deepening search. IEEE Transactions on Pattern Analysis and Machine Intelligence, 16 (7), 701–710.
     *
     * @tparam params_t Type containing the search parameters.
     *
     * @see best_first
     * @see eidastar
     * @see eiddfs
     */
    template<typename params_t>
    struct [[maybe_unused]] eidbest_first final {

        friend class solver<eidbest_first, params_t>;

    private:

        using  index_t = typename params_t:: index_type;
        using scalar_t = typename params_t::scalar_type;
        using  coord_t = typename params_t:: coord_type;
        using solver_t = solver<eidbest_first, params_t>;

        static_assert(std::is_arithmetic_v<scalar_t>, "scalar_t must be an integral or floating point type.");
        static_assert(std::numeric_limits<scalar_t>::is_specialized, "scalar_t must be a numeric type with defined numeric limits.");
        static_assert(std::is_integral_v<index_t>, "index_t must be an integral type.");

        struct node final : bnode<index_t> {

            scalar_t m_hScore;

            /**
             * @brief Constructs an uninitialized node.
             *
             * This constructor creates a node with uninitialized members.
             */
            // ReSharper disable once CppPossiblyUninitializedMember
            [[nodiscard]] constexpr node() noexcept : bnode<index_t>() {} // NOLINT(*-pro-type-member-init, *-use-equals-default)

            [[nodiscard]] constexpr node(index_t _index, scalar_t _hScore) noexcept : bnode<index_t>(_index),
                m_hScore(_hScore) {}

            [[nodiscard]] HOT friend constexpr bool operator < (const node& _a, const node& _b) noexcept {
                return _a.m_hScore > _b.m_hScore;
            }
        };

        template <typename neighbours_t>
        struct state final {

            node curr;
            scalar_t bound;

            neighbours_t neighbours;
            index_t      neighbours_idx;

            [[nodiscard]] state(const node& _curr, scalar_t _bound, const params_t& _params) :
                curr(_curr.m_index, _curr.m_hScore),
                bound(_bound),
                neighbours(_params.maze.get_neighbours(_curr.m_index)),
                neighbours_idx(0U) {}

            ~state() = default;

            state           (const state&) = delete;
            state& operator=(const state&) = delete;

            [[nodiscard]] HOT state(state&&) noexcept = default;

#if __cplusplus > 202302L
            constexpr
#endif
            state& operator=(state&&) noexcept = default;
        };

        using transposition_table_t = std::unordered_map<index_t, scalar_t>;

        template <typename open_set_t>
        [[nodiscard]] HOT static constexpr auto solve_internal(open_set_t& _open, const params_t& _params) {

            using neighbours_t = decltype(_params.maze.get_neighbours(std::declval<index_t>()));

            const auto s = utils::to_1d(_params.start, _params.size);
            const auto e = utils::to_1d(_params.end,   _params.size);

            auto min = std::numeric_limits<scalar_t>::max();

            const auto bound = _params.h(_params.start, _params.end) * _params.weight;

            _open.emplace_back(s, bound);

            stack<state<neighbours_t>> stack{};
            stack.emplace(_open.back(), bound, _params);

            transposition_table_t transposition_table;
            transposition_table[_open.back().m_index] = bound;

            // Main loop:
            while (!stack.empty()) {

                auto& _ = stack.top();
                auto& curr = _.curr;

                if (_.neighbours_idx != _.neighbours.size()) {
                    const auto& n_data = _.neighbours[_.neighbours_idx++];

                    if (const auto& n = solver_t::get_data(n_data, _params); n.active) {

                        auto h = _params.h(n.coord, _params.end) * _params.weight;

                        auto search = transposition_table.find(n.index);
                        if (!(search != transposition_table.end() && h >= search->second)) {
                            transposition_table[n.index] = h;

                            _open.emplace_back(n.index, h);

                            if (n.index != e) { // SEARCH FOR SOLUTION...
                                stack.emplace(_open.back(), _.bound, _params);
                            }
                            else { // SOLUTION REACHED ...

                                // ReSharper disable once CppDFAUnusedValue
                                transposition_table = {};

                                return solver_t::solver_utils::ibacktrack(_open, _params.size);
                            }
                        }
                    }
                }
                else {
                    min = utils::min(min, curr.m_hScore);

                    _open.pop_back();
                    stack.pop();
                }
            }

            return std::vector<coord_t>{};
        }

        [[maybe_unused, nodiscard]] static auto invoke(const params_t& _params) {

            const auto capacity = solver_t::solver_utils::determine_capacity(_params);

            std::pmr::vector<node> open(_params.polytonic_pmr);
            try {
                open.reserve(capacity / 8U);
            }
            catch (...) {} // NOLINT(*-empty-catch)

            return solve_internal(open, _params);
        }
    };

} //chdr::solvers

#endif //CHDR_EID_BEST_FIRST_HPP