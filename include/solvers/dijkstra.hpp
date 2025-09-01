/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_DIJKSTRA_HPP
#define CHDR_DIJKSTRA_HPP

/**
 * @file dijkstra.hpp
 */

#include <iostream>
#include <vector>

#include "base/solver.hpp"

// ReSharper disable once CppUnusedIncludeDirective
#include "../utils/intrinsics.hpp" // NOLINT(*-include-cleaner)

namespace chdr::solvers {

    /**
     * @addtogroup Solvers
     * @brief Graph traversal and pathfinding algorithms.
     * @{
     * @addtogroup Multi-Target
     * @brief Solvers which route to multiple destinations.
     * @{
     * @addtogroup MultiTargetCommon Common
     * @brief General-purpose solvers.
     * @{
     */

    /**
     * @struct dijkstra
     * @brief Dijkstra's algorithm [unfinished].
     * @details Dijkstra's algorithm (Dijkstra, E. W. 1959) is a graph traversal and pathfinding algorithm.
     *          It is a "single-source, multiple-target" (SSMT) algorithm, providing a technique for resolving
     *          the shortest path between a source and every other node in a graph.\n\n
     *
     * @warning This algorithm is not yet implemented.
     *
     * Advantages:
     * - Guarantees the lowest-cost path in graphs with non-negative edge weights.
     * - Effective and efficient in graphs with relatively few edges.
     * - Works well for both directed and undirected graphs.
     *
     * Limitations:
     * - Requires all edge weights to be non-negative (does not handle negative weights).
     * - Can be less efficient in dense graphs compared to other algorithms like Floyd-Warshall.
     * - Quickly consumes memory in large or exhaustive searches.
     * - Largely superseded by other algorithms for "single-source, single-target" (SSST) searches.
     *
     * Further Reading:
     * - <a href="https://en.wikipedia.org/wiki/Dijkstra's_algorithm">Wikipedia Article</a>
     *
     * References:
     * - Dijkstra, E. W., 1959. A Note on Two Problems in Connexion with Graphs. Numerische Mathematik, 1, 269–271.
     *
     * @note Unlike SSST algorithms, Dijkstra returns a shortest-path tree,
     *       indicating the shortest path between any node and the source.
     * @tparam params_t Type containing the search parameters.
     */
    template<typename params_t>
    struct [[maybe_unused]] dijkstra final {

        friend class solver<dijkstra, params_t>;

    private:

        using  index_t = typename params_t:: index_type;
        using scalar_t = typename params_t::scalar_type;
        using  coord_t = typename params_t:: coord_type;
        using solver_t = solver<dijkstra, params_t>;

        struct node final : unmanaged_node<index_t> {

            scalar_t m_gScore;

            /**
             * @brief Constructs an uninitialized node.
             */
            // ReSharper disable once CppPossiblyUninitializedMember
            [[nodiscard]] HOT constexpr node() noexcept : unmanaged_node<index_t>() {} // NOLINT(*-pro-type-member-init, *-use-equals-default)

            [[nodiscard]] HOT constexpr node(index_t _index, scalar_t _gScore, const unmanaged_node<index_t>* RESTRICT const _parent = nullptr) noexcept : unmanaged_node<index_t>(_index, _parent),
                m_gScore(_gScore) {}

            ~node() = default;

            node           (const node&) = delete;
            node& operator=(const node&) = delete;

            [[nodiscard]] HOT constexpr node(node&& _other) noexcept = default;

#if __cplusplus > 202302L
            constexpr
#endif
            HOT node& operator=(node&& _other) noexcept = default;

            [[nodiscard]] HOT friend constexpr bool operator < (const node& _a, const node& _b) noexcept {
                return _a.m_gScore > _b.m_gScore;
            }
        };

        struct paths final {

        private:

            const coord_t m_size;

            std::vector<node> m_memory;

            std::unordered_map<coord_t, node> m_map;

        public:

            [[nodiscard]] HOT constexpr const auto& operator[](const coord_t& _coord) const noexcept {

                auto search = m_map.find(_coord);

                return LIKELY(search != m_map.end()) ?
                    solver_t::solver_utils::rbacktrack(*search, m_size) :
                    std::vector<coord_t>{};
            }
        };

        template <typename open_set_t, typename closed_set_t>
        [[nodiscard]] HOT static constexpr auto solve_internal(open_set_t& _open, closed_set_t& _closed, size_t _capacity, const params_t& _params) {

            const auto s = utils::to_1d(_params.start, _params.size);
            const auto e = utils::to_1d(_params.end,   _params.size);

              _open.emplace_nosort(e, static_cast<scalar_t>(0));
            _closed.emplace(e);

            // Main loop:
            while (LIKELY(!_open.empty())) {

                auto curr(std::move(_open.top()));
                _open.pop();

                if (curr.m_index != s) { // SEARCH FOR SOLUTION...

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
                                    _open.emplace_nosort(n.index, curr_ptr->m_gScore + n.distance, curr_ptr);
                                }
                                else {
                                    _open.emplace(n.index, curr_ptr->m_gScore + n.distance, curr_ptr);
                                }
                            }
                        }
                    }
                }
                else { // SOLUTION REACHED...

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

#endif //CHDR_DIJKSTRA_HPP