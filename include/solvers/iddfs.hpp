/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_IDDFS_HPP
#define CHDR_IDDFS_HPP

/**
 * @file iddfs.hpp
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
     * @struct iddfs
     * @brief Iterative-deepening depth-first search algorithm.
     * @details IDDFS* (Korf, R. E., 1985) is a graph traversal and pathfinding algorithm that repeatedly explores
     *          the search space in a depth-first manner with an incrementing depth limit.\n\n
     *
     * Advantages:
     * - Minimises memory usage by not maintaining a record of the search state.
     * - Does not need a pre-pass, although performance can improve if the search space is pruned first.
     *
     * Limitations:
     * - Low performance due to repeated traversal of the search space.
     *
     * Further Reading:
     * - <a href="https://en.wikipedia.org/wiki/Iterative_deepening_depth-first_search">Wikipedia Article</a>
     *
     * References:
     * - Korf, R. E., 1985. Depth-first iterative-deepening. Artificial Intelligence, 27 (1), 97–109.
     *
     * @note Similarly to BFS, the resulting path is optimal if the search space is uniform-cost.
     *
     * @tparam params_t Type containing the search parameters.
     *
     * @see dfs
     * @see bfs
     * @see eiddfs
     */
    template<typename params_t>
    struct [[maybe_unused]] iddfs final {

        friend class solver<iddfs, params_t>;

    private:

        using  index_t = typename params_t::index_type;
        using  coord_t = typename params_t::coord_type;
        using solver_t = solver<iddfs, params_t>;

        static_assert(std::is_integral_v<index_t>, "index_t must be an integral type.");

        struct node final : bnode<index_t> {

            index_t m_depth;

            /**
             * @brief Constructs an uninitialized node.
             */
            // ReSharper disable once CppPossiblyUninitializedMember
            [[nodiscard]] constexpr node() noexcept : bnode<index_t>() {} // NOLINT(*-pro-type-member-init, *-use-equals-default)

            [[nodiscard]] constexpr node(index_t _index, index_t _depth) noexcept : bnode<index_t>(_index),
                m_depth(_depth) {}

            [[nodiscard]] HOT friend constexpr bool operator < (const node& _a, const node& _b) noexcept {
                return _a.m_hScore > _b.m_hScore;
            }
        };

        template <typename neighbours_t>
        struct state final {

            neighbours_t neighbours;
            index_t      neighbours_idx;

            [[nodiscard]] state(const node& _curr, const params_t& _params) :
                neighbours(_params.maze.get_neighbours(_curr.m_index)),
                neighbours_idx(0U) {}

            ~state() = default;

            state           (const state&) = delete;
            state& operator=(const state&) = delete;

            [[nodiscard]] HOT constexpr state(state&&) noexcept = default;

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

            _open.emplace_back(s, 0U);

            stack<state<neighbours_t>> stack{};

            for (size_t bound = 0U; bound < std::numeric_limits<size_t>::max(); ++bound) {

                stack.emplace(_open.back(), _params);

                // Main loop:
                while (!stack.empty()) {

                    auto& _    = stack.top();
                    auto& curr = _open.back();

                    if (curr.m_depth <= bound && _.neighbours_idx != _.neighbours.size()) {

                        if (const auto& n = solver_t::get_data(_.neighbours[_.neighbours_idx++], _params); n.active) {

                            if (std::none_of(_open.begin(), _open.end(), [&n](const auto& _item) ALWAYS_INLINE { return _item.m_index == n.index; })) {

                                _open.emplace_back(n.index, curr.m_depth + 1U);

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

                _open.erase(_open.begin() + 1U, _open.end());
                stack.clear();
            }

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

#endif //CHDR_IDDFS_HPP