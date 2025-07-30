/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_IDASTAR_HPP
#define CHDR_IDASTAR_HPP

/**
 * @file idastar.hpp
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
     * @struct idastar
     * @brief Iterative-deepening A* search algorithm.
     * @details IDA* (Korf, R. E., 1985) is a heuristic-informed variant of the iterative-deepening DFS algorithm.\n
     *          It minimises memory usage by repeatedly traversing the search space with incrementing cost thresholds.\n\n
     *
     * Advantages:
     * - Heuristic-driven search can improve search times when compared to IDDFS.
     * - Minimises memory usage by not maintaining a record of the search state.
     * - Does not need a pre-pass, although performance can improve if the search space is pruned first.
     *
     * Limitations:
     * - Low performance due to repeated traversal of the search space.
     *
     * Further Reading:
     * - <a href="https://en.wikipedia.org/wiki/Iterative_deepening_A*">Wikipedia Article</a>
     *
     * References:
     * - Korf, R. E., 1985. Depth-first iterative-deepening. Artificial Intelligence, 27 (1), 97–109.
     *
     * @note Guarantees the optimal path if the heuristic is admissible (never overestimates the cost).
     *
     * @tparam params_t Type containing the search parameters.
     *
     * @see astar
     * @see gstar
     * @see iddfs
     * @see eidastar
     */
    template<typename params_t>
    struct [[maybe_unused]] idastar final {

        friend class solver<idastar, params_t>;

    private:

        using  index_t = typename params_t:: index_type;
        using scalar_t = typename params_t::scalar_type;
        using  coord_t = typename params_t:: coord_type;
        using solver_t = solver<idastar, params_t>;

        struct node final : bnode<index_t> {

            scalar_t m_gScore;
            scalar_t m_fScore;

            /**
             * @brief Constructs an uninitialized node.
             */
            // ReSharper disable once CppPossiblyUninitializedMember
            [[nodiscard]] constexpr node() noexcept : bnode<index_t>() {} // NOLINT(*-pro-type-member-init, *-use-equals-default)

            [[nodiscard]] constexpr node(index_t _index, scalar_t _gScore, scalar_t _fScore) noexcept : bnode<index_t>(_index),
                m_gScore(_gScore),
                m_fScore(_fScore) {}

            [[nodiscard]] HOT friend constexpr bool operator < (const node& _a, const node& _b) noexcept {
                return _a.m_fScore == _b.m_fScore ?
                       _a.m_gScore >  _b.m_gScore :
                       _a.m_fScore >  _b.m_fScore;
            }
        };

        template <typename neighbours_t>
        struct state final {

            node curr;

            neighbours_t neighbours;
            size_t       neighbours_idx;

            [[nodiscard]] state(const node& _curr, const params_t& _params) :
                curr(_curr.m_index, _curr.m_gScore, _curr.m_fScore),
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

            _open.emplace_back(s, 0, bound);

            stack<state<neighbours_t>> stack(_params.heterogeneous_pmr);

            do {

                stack.emplace(_open.back(), _params);

                auto min = std::numeric_limits<scalar_t>::max();

                // Main loop:
                while (!stack.empty()) {

                    auto& _    = stack.top();
                    auto& curr = _.curr;

                    if (curr.m_fScore <= bound) {

                        if (_.neighbours_idx != _.neighbours.size()) {

                            if (const auto& n = solver_t::get_data(_.neighbours[(_.neighbours.size() - 1U) - (_.neighbours_idx++)], _params); n.active) {

                                if (std::none_of(_open.begin(), _open.end(), [&n](const auto& _item) ALWAYS_INLINE { return _item.m_index == n.index; })) {

                                    auto g = curr.m_gScore + n.distance;
                                    auto f = g + (_params.h(n.coord, _params.end) * _params.weight);

                                    _open.emplace_back(n.index, g, f);

                                    if (n.index != e) { // SEARCH FOR SOLUTION...
                                        stack.emplace(_open.back(), _params);
                                    }
                                    else { // SOLUTION REACHED...
                                        return solver_t::solver_utils::ibacktrack(_open, _params.size);
                                    }
                                }
                            }
                        }
                        else {
                            if (_open.size() > 1U) {
                                _open.pop_back();
                            }
                            stack.pop();
                        }
                    }
                    else {
                        min = utils::min(min, curr.m_fScore);
                        if (_open.size() > 1U) {
                            _open.pop_back();
                        }
                        stack.pop();
                    }
                }

                bound = min;

                stack.clear();
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

#endif //CHDR_IDASTAR_HPP