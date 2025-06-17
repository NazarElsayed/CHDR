/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_SMASTAR_HPP
#define CHDR_SMASTAR_HPP

/**
 * @file smastar.hpp
 */

#include <cstddef>
#include <type_traits>
#include <vector>
#include <set>

#include "../types/containers/heap.hpp"
#include "../utils/utils.hpp"
#include "base/solver.hpp"
#include "base/mutable_node.hpp"

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
     * @addtogroup SingleTargetCommon Common
     * @brief General-purpose solvers.
     * @{
     */

    /**
     * @struct smastar
     * @brief SMA* search algorithm.
     *
     * @tparam params_t Type containing the search parameters.
     */
    template<typename params_t>
    struct [[maybe_unused]] smastar final {

        friend class solver<smastar, params_t>;

    private:

        using  index_t = typename params_t:: index_type;
        using scalar_t = typename params_t::scalar_type;
        using  coord_t = typename params_t:: coord_type;
        using solver_t = solver<smastar, params_t>;

        static_assert(std::is_arithmetic_v<scalar_t>, "scalar_t must be an integral or floating point type.");
        static_assert(std::is_integral_v<index_t>, "index_t must be an integral type.");

        struct node final : mutable_node<index_t> {

            bool m_in_path;

             index_t m_depth;
            scalar_t m_gScore;
            scalar_t m_fScore;

            std::vector<index_t> m_successors;

            /**
             * @brief Constructs an uninitialized node.
             */
            // ReSharper disable once CppPossiblyUninitializedMember
            [[nodiscard]] HOT constexpr node() noexcept : mutable_node<index_t>() {} // NOLINT(*-pro-type-member-init, *-use-equals-default)

            [[nodiscard]] HOT constexpr node(index_t _depth, index_t _index, scalar_t _gScore, scalar_t _hScore, mutable_node<index_t>* RESTRICT const _parent = nullptr) noexcept : mutable_node<index_t>(_index, _parent),
                m_in_path   (false),
                m_depth     (_depth           ),
                m_gScore    (_gScore          ),
                m_fScore    (_gScore + _hScore),
                m_successors() {}

            ~node() = default;

            node           (const node&) = default;
            node& operator=(const node&) = default;

            [[nodiscard]] HOT constexpr node(node&& _other) noexcept = default;

#if __cplusplus > 202302L
            constexpr
#endif
            HOT node& operator=(node&& _other) noexcept = default;

            [[nodiscard]] HOT friend constexpr bool operator < (const node& _a, const node& _b) noexcept {
                return _a.m_fScore == _b.m_fScore ?
                       _a.m_gScore >  _b.m_gScore :
                       _a.m_fScore >  _b.m_fScore;
            }
        };

        template <typename open_set_t>
        [[nodiscard]] HOT static constexpr auto solve_internal(open_set_t& _open, const params_t& _params) {

            const auto s = utils::to_1d(_params.start, _params.size);
            const auto e = utils::to_1d(_params.end,   _params.size);

            std::unordered_map<index_t, node> all_nodes;

            {
                auto& start_node = all_nodes[static_cast<index_t>(s)];
                start_node = node(static_cast<index_t>(0), static_cast<index_t>(s), static_cast<scalar_t>(0), _params.h(_params.start, _params.end) * _params.weight);
                start_node.m_in_path = true;

                _open.emplace(start_node.m_index);
            }

            // Main loop:
            while (LIKELY(!_open.empty())) {

                auto& curr = all_nodes[_open.top()];
                _open.pop();

                if (curr.m_index == e) { // SOLUTION REACHED...
                    return solver_t::solver_utils::rbacktrack(curr, _params.size, curr.m_depth);
                }
                else {
                    // SEARCH FOR SOLUTION...

                    // TODO:
                    std::cout << "Current node index: " << curr.m_index << '\n';

                    // Expand successors if not already expanded:
                    if (curr.m_successors.empty()) {

                        for (const auto& n_data : _params.maze.get_neighbours(curr.m_index)) {

                            if (const auto& n = solver_t::get_data(n_data, _params); n.active) {

                                auto g = curr.m_gScore + n.distance;
                                auto h = _params.h(n.coord, _params.end) * _params.weight;

                                // Create or update successor node:
                                auto child_search = all_nodes.find(n.index);
                                if (child_search == all_nodes.end()) {
                                    all_nodes[n.index] = node(curr.m_depth + 1, n.index, g, h, &curr);
                                }
                                else {
                                    auto& child_node = child_search->second;
                                    if (!child_node.m_in_path && g < child_node.m_gScore) {
                                         child_node.m_gScore = g;
                                         child_node.m_fScore = g + h;
                                         child_node.m_parent = &curr;
                                    }
                                }

                                curr.m_successors.push_back(n.index);
                                _open.emplace(n.index);
                            }
                        }
                    }

                    // Select the best successor
                    node* best_successor = nullptr;
                    for (const auto& succ_idx : curr.m_successors) {

                        auto& succ = all_nodes[succ_idx];
                        if (!succ.m_in_path && (best_successor == nullptr || succ.m_fScore < best_successor->m_fScore)) {
                            best_successor = &succ;
                        }
                    }

                    if (best_successor != nullptr) {
                        best_successor->m_in_path = true;
                    }
                    else {

                        // No valid successor found, back up fScore.
                        scalar_t min_f = std::numeric_limits<scalar_t>::max();
                        for (const auto& succ_idx : curr.m_successors) {
                            auto& succ = all_nodes[succ_idx];
                            min_f = std::min(min_f, succ.m_fScore);
                        }
                        curr.m_fScore = std::max(curr.m_fScore, min_f);

                        if (curr.m_parent != nullptr) {
                            auto* p = static_cast<node*>(curr.m_parent);
                            p->m_fScore = std::max(p->m_fScore, curr.m_fScore);
                        }
                    }

                    // TODO:
                    std::cout << "Open set size: " << _open.size() << '\n';

                    // If memory is exhausted, purge worst node.
                    if (_open.size() > _params.memory_limit) {
                        auto worst_itr = std::max_element(
                            _open.cbegin(),
                            _open.cend(),
                            [&](const index_t& a_idx, const index_t& b_idx) {
                                const auto& a = all_nodes.at(a_idx);
                                const auto& b = all_nodes.at(b_idx);
                                return a.m_fScore < b.m_fScore;
                            }
                        );

                        if (worst_itr != _open.end()) {
                            const index_t worst_idx = *worst_itr;
                            node& worst_node = all_nodes[worst_idx];

                            if (!worst_node.m_in_path) {
                                _open.erase(worst_idx);      // Remove index from open set
                                all_nodes.erase(worst_idx);  // Remove node from all_nodes
                            }
                        }
                    }
                }
            }

            return std::vector<coord_t>{};
        }

        [[maybe_unused, nodiscard]] static auto invoke(const params_t& _params) {

            assert(_params.memory_limit > 0U && "memory_limit must be greater than zero.");

            if (_params.memory_limit > 0U) {
                heap<index_t> open(_params.heterogeneous_pmr);

                return solve_internal(open, _params);
            }
            else {
                return std::vector<coord_t>{};
            }
        }
    };

    /**
     * @}
     * @}
     * @}
     */

} //chdr::solvers

#endif //CHDR_ASTAR_HPP