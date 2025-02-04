/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_BEST_FIRST_HPP
#define CHDR_BEST_FIRST_HPP

/**
 * @file best_first.hpp
 */

#include <cstddef>
#include <type_traits>
#include <vector>

#include "../types/containers/existence_set.hpp"
#include "../types/containers/heap.hpp"
#include "../utils/utils.hpp"
#include "base/solver.hpp"
#include "base/unmanaged_node.hpp"

// ReSharper disable once CppUnusedIncludeDirective
#include "../utils/intrinsics.hpp" // NOLINT(*-include-cleaner)

namespace chdr::solvers {

    /**
     * @addtogroup Solvers
     * @{
     * @addtogroup Single-Target
     * @brief Solvers which route to a single destination.
     * @{
     * @addtogroup SingleTargetCommon Common
     * @brief General-purpose solvers.
     * @{
     */

    /**
     * @struct best_first
     * @brief Greedy best-first search algorithm.
     * @details A heuristic-informed graph traversal and pathfinding algorithm that always prioritises its next move
     *          based on least cost.\n\n
     *
     * Advantages:
     * - Lower constant time and memory factor than A*.
     * - Highly effective in searches with few obstacles.
     * - Does not need a pre-pass, although performance can improve if the search space is pruned first.
     * - High performance in bounded (finite) search scenarios.
     *
     * Limitations:
     * - Does not guarantee an optimal path according to the cost heuristic.
     * - Quickly consumes memory in large or exhaustive searches.
     * - Inefficient or complex search heuristics can reduce performance.
     * - Poor performance when searches lack solutions.
     *
     * Further Reading:
     * - <a href="https://en.wikipedia.org/wiki/Best-first_search">Wikipedia Article</a>
     *
     * @tparam params_t Type containing the search parameters.
     */
    template<typename params_t>
    struct [[maybe_unused]] best_first final {

        friend class solver<best_first, params_t>;

    private:

        using  index_t = typename params_t:: index_type;
        using scalar_t = typename params_t::scalar_type;
        using  coord_t = typename params_t:: coord_type;
        using solver_t = solver<best_first, params_t>;

        static_assert(std::is_arithmetic_v<scalar_t>, "scalar_t must be an integral or floating point type.");
        static_assert(std::is_integral_v<index_t>, "index_t must be an integral type.");

        struct node final : unmanaged_node<index_t> {

            scalar_t m_hScore;

            /**
             * @brief Constructs an uninitialized node.
             */
            // ReSharper disable once CppPossiblyUninitializedMember
            [[nodiscard]] constexpr node() noexcept {} // NOLINT(*-pro-type-member-init, *-use-equals-default)

            [[nodiscard]] constexpr node(index_t _index, scalar_t _hScore, const unmanaged_node<index_t>* RESTRICT const _parent = nullptr) noexcept : unmanaged_node<index_t>(_index, _parent),
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
        [[nodiscard]] HOT static constexpr auto solve_internal(open_set_t& _open, closed_set_t& _closed, size_t _capacity, const params_t& _params) {

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
                                    curr_ptr = new (_params.monotonic_pmr->allocate(sizeof(node), alignof(node))) node(std::move(curr));
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
                }
                else { // SOLUTION REACHED ...

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

#endif //CHDR_BEST_FIRST_HPP