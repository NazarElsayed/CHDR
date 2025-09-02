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
#include "include/types/pmr/monotonic_pool.hpp"

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
     *          the shortest path between one source and every other node in a graph.\n\n
     *
     * @warning This implementation of Dijkstra's algorithm is not yet finished.
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

            node           (const node&) = default;
            node& operator=(const node&) = default;

            [[nodiscard]] HOT constexpr node(node&& _other) noexcept = default;

#if __cplusplus > 202302L
            constexpr
#endif
            HOT node& operator=(node&& _other) noexcept = default;

            [[nodiscard]] HOT friend constexpr bool operator < (const node& _a, const node& _b) noexcept {
                return _a.m_gScore > _b.m_gScore;
            }
        };

        /**
         * @struct multi_result
         * @brief Encapsulates a predecessor map for Dijkstra's algorithm results.
         * @details This structure maintains a mapping of nodes to their predecessors,
         *          allowing for path reconstruction from any reached node back to the source.
         *          It uses a monotonic memory resource for efficient memory allocation
         *          during the path construction process.
         */
        struct multi_result {

            friend dijkstra;

        private:

            coord_t m_size;

            // ReSharper disable once CppRedundantQualifier
            chdr::monotonic_pool<> m_resource;

            std::pmr::unordered_map<size_t, node> m_data;

        public:

            // ReSharper disable once CppRedundantMemberInitializer
            multi_result(coord_t _size, size_t _capacity = 0U) :
                m_size     (_size),
                m_resource (),
                m_data     (&m_resource)
            {
                m_data.reserve(_capacity);
            }

            HOT auto get(coord_t _coord) const {

                std::vector<coord_t> result;

                if (const auto search = m_data.find(utils::to_1d(_coord, m_size)); search != m_data.end()) {

                    result.emplace_back(_coord);

                    for (const auto* RESTRICT t = search->second.m_parent; t != nullptr; t = t->m_parent) {
                        result.emplace_back(utils::to_nd(t->m_index, m_size));
                    }
                }

                return result;
            }
        };

        template <typename open_set_t>
        [[nodiscard]] HOT static constexpr auto solve_internal(open_set_t& _open, const params_t& _params) {

            const auto e = utils::to_1d(_params.end, _params.size);

            multi_result result(_params.size, _params.capacity);

            _open.emplace_nosort(e, static_cast<scalar_t>(0));

            // Main loop:
            while (LIKELY(!_open.empty())) {

                auto _ = std::move(_open.top());
                _open.pop();

                auto& curr = result.m_data.emplace(_.m_index, std::move(_)).first->second;

                for (const auto& n_data : _params.maze.get_neighbours(curr.m_index)) {

                    if (const auto& n = solver_t::get_data(n_data, _params); n.active) {

                        auto g = curr.m_gScore + n.distance;

                        if (const auto search = result.m_data.find(n.index); search != result.m_data.end()) {

                            if (auto& child = search->second; g < child.m_gScore) {

                                child.m_gScore = g;
                                child.m_parent = &curr;

                                if constexpr (params_t::lazy_sorting::value) {
                                    _open.emplace_nosort(child);
                                }
                                else {
                                    _open.emplace(child);
                                }
                            }
                        }
                        else {
                            if constexpr (params_t::lazy_sorting::value) {
                                _open.emplace_nosort(n.index, g, &curr);
                            }
                            else {
                                _open.emplace(n.index, g, &curr);
                            }
                        }
                    }
                }
            }

            // TODO: return multi_result instead of std::vector<coord_t>.
            return result.get(_params.start);
        }

        [[maybe_unused, nodiscard]] static auto invoke(const params_t& _params) {

            const auto capacity = solver_t::solver_utils::determine_capacity(_params);

            heap<node> open(_params.heterogeneous_pmr);
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

#endif //CHDR_DIJKSTRA_HPP