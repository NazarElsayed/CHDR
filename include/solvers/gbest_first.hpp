/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_GBEST_FIRST_HPP
#define CHDR_GBEST_FIRST_HPP

/**
 * @file gbest_first.hpp
 */

#include <cstddef>

#include "../solvers/base/managed_node.hpp"
#include "../types/containers/existence_set.hpp"
#include "../types/containers/heap.hpp"
#include "../utils/utils.hpp"
#include "base/solver.hpp"

// ReSharper disable once CppUnusedIncludeDirective
#include "base/managed_node.hpp" // NOLINT(*-include-cleaner)

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
     * @addtogroup SingleTargetGraveyardOptimised Graveyard-Optimised
     * @brief Graveyard solvers, which dynamically prune the search tree.
     * @{
     */

    /**
     * @struct gbest_first
     * @brief Graveyard-optimised variant of the best-first search algorithm.
     * @details G*-optimised variant of best-first search, reducing memory usage by allowing for the
     *          dynamic pruning of redundant data from the search tree.
     *
     * Advantages:
     * - Significantly reduced average memory usage when compared to best-first search.
     * - Capable of outperforming best-first search if the overhead due to memory allocation is lower.
     *
     * Limitations:
     * - Higher constant factor than best-first search makes it less effective in small searches.
     *
     * @tparam params_t Type containing the search parameters.
     * @see best_first
     * @see gstar
     */
    template<typename params_t>
    struct [[maybe_unused]] gbest_first final {

        friend class solver<gbest_first, params_t>;

    private:

        using  index_t = typename params_t:: index_type;
        using scalar_t = typename params_t::scalar_type;
        using  coord_t = typename params_t:: coord_type;
        using solver_t = solver<gbest_first, params_t>;

        struct node final : managed_node<index_t, node> {

            scalar_t m_hScore;

            /**
             * @brief Constructs an uninitialized node.
             */
            // ReSharper disable once CppPossiblyUninitializedMember
            [[nodiscard]] constexpr node() noexcept : managed_node<index_t, node>() {} // NOLINT(*-pro-type-member-init, *-use-equals-default)

            [[nodiscard]] constexpr node(index_t _index, scalar_t _hScore, node* RESTRICT const _parent = nullptr) noexcept : managed_node<index_t, node>(_index, _parent),
                m_hScore(_hScore) {}

            ~node() = default;

            node           (const node&) = delete;
            node& operator=(const node&) = delete;

            [[nodiscard]] HOT constexpr node(node&& _other) noexcept = default;

#if __cplusplus > 202302L
            constexpr
#endif
            HOT node& operator=(node&& _other) noexcept = default;

            [[nodiscard]] HOT friend constexpr bool operator < (const node& _a, const node& _b) noexcept {
                return _a.m_hScore > _b.m_hScore;
            }
        };

        template <typename open_set_t, typename closed_set_t>
        [[maybe_unused, nodiscard]] HOT static constexpr auto solve_internal(open_set_t& _open, closed_set_t& _closed, size_t _capacity, const params_t& _params) {

            const auto s = utils::to_1d(_params.start, _params.size);
            const auto e = utils::to_1d(_params.end,   _params.size);

              _open.emplace_nosort(s, _params.h(_params.start, _params.end));
            _closed.emplace(s);

            // Main loop:
            while (LIKELY(!_open.empty())) {

                auto curr(std::move(_open.top()));
                _open.pop();

                if (curr.m_index != e) { // SEARCH FOR SOLUTION...

                    node* RESTRICT curr_ptr(nullptr);

                    for (const auto& n_data : _params.maze.get_neighbours(curr.m_index)) {

                        if (const auto& n = solver_t::get_data(n_data, _params); n.active) {

                            // Check if node is not already visited:
                            if (!_closed.contains(n.index)) {
                                solver_t::solver_utils::preallocate_emplace(_closed, n.index, _capacity, _params.maze.count());

                                if (curr_ptr == nullptr) {
                                    curr_ptr = new (_params.homogeneous_pmr->allocate(sizeof(node), alignof(node))) node(std::move(curr));
                                }

                                if constexpr (params_t::lazy_sorting::value) {
                                    _open.emplace_nosort(n.index, _params.h(n.coord, _params.end), curr_ptr);
                                }
                                else {
                                    _open.emplace(n.index, _params.h(n.coord, _params.end), curr_ptr);
                                }
                            }
                        }
                    }

                    if (curr_ptr == nullptr) {
                        curr.expunge(_params.homogeneous_pmr);
                    }
                }
                else { // SOLUTION REACHED...

                    if constexpr (std::is_same_v<std::decay_t<decltype(_open)>, heap<node>>) {
                        _open.wipe();
                    }
                    else {
                        _open = open_set_t{};
                    }
                    _closed = closed_set_t{};

                    return solver_t::solver_utils::rbacktrack(curr, _params.size);
                }
            }

            return std::vector<coord_t>{};
        }

        [[maybe_unused, nodiscard]] static auto invoke(const params_t& _params) {

            const auto capacity = solver_t::solver_utils::determine_capacity(_params);

            existence_set closed(_params.monotonic_pmr);
            closed.reserve(capacity);

            heap<node> open(_params.heterogeneous_pmr);
            try {
                open.reserve(capacity / 8U);
            }
            catch (...) {} // NOLINT(*-empty-catch)

            return solve_internal(open, closed, capacity, _params);
        }
    };

    /**
     * @}
     * @}
     * @}
     */

} //chdr::solvers

#endif //CHDR_GBEST_FIRST_HPP