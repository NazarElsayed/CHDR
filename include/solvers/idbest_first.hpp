/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_ID_BEST_FIRST_HPP
#define CHDR_ID_BEST_FIRST_HPP

/**
 * @file idbest_first.hpp
 */

#include <cstddef>
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
     * @addtogroup Solvers
     * @brief Graph traversal and pathfinding algorithms.
     * @{
     * @addtogroup Single-Target
     * @brief Solvers which route to a single destination.
     * @{
     * @addtogroup SingleTargetIterativeDeepening Iterative-Deepening
     * @brief Solvers which repeatedly traverse the search space at incrementally-increasing depths.
     * @{
     */

    /**
     * @struct idbest_first
     * @brief Iterative-deepening best-first search algorithm.
     * @details Iterative-deepening best-first search is a heuristic-informed variant of the
     *          iterative-deepening DFS algorithm (Korf, R. E., 1985).\n
     *          It minimises memory usage by repeatedly traversing the search space with incrementing cost thresholds.\n\n
     *
     * Advantages:
     * - Heuristic-driven search can improve search times when compared to IDDFS.
     * - Minimises memory usage by not maintaining a record of the search state.
     * - Does not need a pre-pass, although performance can improve if the search space is pruned first.
     *
     * Limitations:
     * - Does not guarantee an optimal path according to the cost heuristic.
     * - Low performance due to repeated traversal of the search space.
     *
     * Further Reading:
     * - <a href="https://en.wikipedia.org/wiki/Iterative_deepening_depth-first_search">Wikipedia — Iterative Deepening Depth-First Search</a>
     * - <a href="https://en.wikipedia.org/wiki/Best-first_search">Wikipedia — Best First Search</a>
     *
     * References:
     * - Korf, R. E., 1985. Depth-first iterative-deepening. Artificial Intelligence, 27 (1), 97–109.
     *
     * @tparam params_t Type containing the search parameters.
     *
     * @see best_first
     * @see iddfs
     * @see eidbest_first
     */
    template<typename params_t>
    struct [[maybe_unused]] idbest_first final {

        friend class solver<idbest_first, params_t>;

    private:

        using  index_t = typename params_t:: index_type;
        using scalar_t = typename params_t::scalar_type;
        using  coord_t = typename params_t:: coord_type;
        using solver_t = solver<idbest_first, params_t>;

        static_assert(std::is_arithmetic_v<scalar_t>, "scalar_t must be an integral or floating point type.");
        static_assert(std::numeric_limits<scalar_t>::is_specialized, "scalar_t must be a numeric type with defined numeric limits.");
        static_assert(std::is_integral_v<index_t>, "index_t must be an integral type.");

        struct node final : bnode<index_t> {

            scalar_t m_hScore;

            /**
             * @brief Constructs an uninitialized node.
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

            neighbours_t neighbours;
            size_t       neighbours_idx;

            [[nodiscard]] state(const node& _curr, const params_t& _params) :
                curr(_curr.m_index, _curr.m_hScore),
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

        template <typename open_set_t>
        [[nodiscard]] HOT static constexpr auto solve_internal(open_set_t& _open, const params_t& _params) {

            using neighbours_t = decltype(_params.maze.get_neighbours(std::declval<index_t>()));

            const auto s = utils::to_1d(_params.start, _params.size);
            const auto e = utils::to_1d(_params.end,   _params.size);

            auto bound = _params.h(_params.start, _params.end) * _params.weight;

            _open.emplace_back(s, bound);

            stack<state<neighbours_t>> stack{};

            do {

                stack.emplace(_open.back(), _params);

                auto min = std::numeric_limits<scalar_t>::max();

                // Main loop:
                while (!stack.empty()) {

                    auto& _    = stack.top();
                    auto& curr = _.curr;

                    if (curr.m_hScore <= bound) {

                        if (_.neighbours_idx != _.neighbours.size()) {

                            if (const auto& n = solver_t::get_data(_.neighbours[(_.neighbours.size() - 1U) - (_.neighbours_idx++)], _params); n.active) {

                                if (std::none_of(_open.begin(), _open.end(), [&n](const auto& _item) ALWAYS_INLINE { return _item.m_index == n.index; })) {

                                    auto h = _params.h(n.coord, _params.end) * _params.weight;

                                    _open.emplace_back(n.index, h);

                                    if (n.index != e) { // SEARCH FOR SOLUTION...
                                        stack.emplace(_open.back(), _params);
                                    }
                                    else { // SOLUTION REACHED ...
                                        return solver_t::solver_utils::ibacktrack(_open, _params.size);
                                    }
                                }
                            }
                        }
                        else {
                            _open.pop_back();
                            stack.pop();
                        }
                    }
                    else {
                        min = utils::min(min, curr.m_hScore);
                    }
                }

                _open.erase(_open.begin() + 1U, _open.end());
                stack.clear();

                bound = min;
            }
            while (bound != std::numeric_limits<scalar_t>::max());

            return std::vector<coord_t>{};
        }

        [[maybe_unused, nodiscard]] static auto invoke(const params_t& _params) {

            const auto capacity = solver_t::solver_utils::determine_capacity(_params);

            std::pmr::vector<node> open(_params.heterogeneous_pmr);
            try {
                open.reserve(capacity / 8U);
            }
            catch (...) {} // NOLINT(*-empty-catch)

            return solve_internal(open, _params);
        }
    };

    /**
     * @}
     * @}
     * @}
     */

} //chdr::solvers

#endif //CHDR_ID_BEST_FIRST_HPP