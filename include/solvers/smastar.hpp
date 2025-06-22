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

        static constexpr auto null_v = std::numeric_limits<index_t>::max();

        struct node final {

             index_t m_depth;
             index_t m_index;
            scalar_t m_gScore;
            scalar_t m_fScore;
             index_t m_parent;

            std::vector<index_t> m_successors;

            /**
             * @brief Constructs an uninitialized node.
             */
            // ReSharper disable once CppPossiblyUninitializedMember
            [[nodiscard]] HOT constexpr node() noexcept {} // NOLINT(*-pro-type-member-init, *-use-equals-default)

            [[nodiscard]] HOT constexpr node(index_t _depth, index_t _index, scalar_t _gScore, scalar_t _hScore, index_t _parent = null_v) noexcept :
                m_depth     (_depth           ),
                m_index     (_index           ),
                m_gScore    (_gScore          ),
                m_fScore    (_gScore + _hScore),
                m_parent    (_parent          ),
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

        template <typename open_set_t, typename all_nodes_t>
        static size_t memory_usage(const open_set_t& _open, const all_nodes_t& _all_nodes) {
            return _all_nodes.size();
        }

        template <typename open_set_t, typename all_nodes_t>
        static auto remove_weakest(open_set_t& _open, all_nodes_t& _all_nodes, index_t _forbidden = null_v) {

            assert(!_all_nodes.empty() && "_all_nodes is empty!");

            // Find least-useful node.
            //const auto weakest_open = std::prev(_open.end());

            // Find deepest node (maximum depth) with highest f-score.
            auto weakest_all = _all_nodes.begin();
            for (auto it = _all_nodes.begin(); it != _all_nodes.end(); ++it) {
                if (it->second.m_index != _forbidden &&
                    (
                         it->second.m_depth > weakest_all->second.m_depth ||
                        (it->second.m_depth == weakest_all->second.m_depth &&
                         it->second.m_fScore > weakest_all->second.m_fScore)
                    )
                ) {
                    weakest_all = it;
                }
            }

            auto weakest = weakest_all->second;
            if (weakest.m_index != _forbidden) {

                std::cout << "Memory usage: " << memory_usage(_open, _all_nodes) << "\tRemoving: " << weakest_all->second.m_index << "\n";

                auto it = std::find_if(
                    _open.begin(),
                    _open.end(),
                    [&weakest](const auto& item) { return item == weakest.m_index; }
                );
                if (it != _open.end()) { _open.erase(it); }

                backup_f_values(weakest_all->second, _all_nodes);
                _all_nodes.erase(weakest_all);

                return true;
            }

            return false;
        }

        template <typename all_nodes_t>
        static void backup_f_values(const node& _removed_node, all_nodes_t& _all_nodes) {

            auto current_parent = _removed_node.m_parent;
            while (current_parent != null_v) {

                auto parent_it = _all_nodes.find(current_parent);
                if (parent_it == _all_nodes.end()) {
                    break;
                }

                auto& parent_node = parent_it->second;
                auto min_child_f  = std::numeric_limits<scalar_t>::max();
                bool has_children = false;

                // Find minimum f-value amongst children.
                for (const auto& child_idx : parent_node.m_successors) {
                    auto child_it = _all_nodes.find(child_idx);
                    if (child_it != _all_nodes.end()) {
                        min_child_f  = std::min(min_child_f, child_it->second.m_fScore);
                        has_children = true;
                    }
                }

                // Update parent's f-value if we found children.
                if (has_children && min_child_f > parent_node.m_fScore) {
                    parent_node.m_fScore = min_child_f;
                    current_parent = parent_node.m_parent; // Move up the chain
                }
                else {
                    break;
                }
            }
        }

        [[nodiscard]] HOT static constexpr auto solve_internal(const params_t& _params) {

            const auto s = utils::to_1d(_params.start, _params.size);
            const auto e = utils::to_1d(_params.end,   _params.size);

            std::unordered_map<index_t, node> all_nodes;

            const static auto node_comparator = [&all_nodes](index_t _a, index_t _b) {

                auto it_a = all_nodes.find(_a);
                auto it_b = all_nodes.find(_b);

                assert(!(it_a == all_nodes.end() || it_b == all_nodes.end()) && "Node lookup failed in comparator");

                return !(it_a->second < it_b->second);
            };

            std::set<index_t, decltype(node_comparator)> open(node_comparator);

            // Add start node
            all_nodes[s] = node(
                static_cast<index_t>(0),
                static_cast<index_t>(s),
                static_cast<scalar_t>(0),
                _params.h(_params.start, _params.end) * _params.weight
            );
            open.emplace(static_cast<index_t>(s));

            // Main loop:
            while (LIKELY(!open.empty())) {

                std::cout << "Memory usage: " << memory_usage(open, all_nodes) << "\n";

                // Handle memory management:
                while (open.size() > 1U && memory_usage(open, all_nodes) >= _params.memory_limit) {
                    remove_weakest(open, all_nodes);
                }

                // Get current node index and remove from open set
                auto curr_idx = *open.begin();
                open.erase(open.begin());

                auto& curr = all_nodes[curr_idx];

                if (curr.m_index == e) { // SOLUTION REACHED...

                    std::vector<coord_t> result(curr.m_depth);

                    size_t i = 0U;
                    for (auto p = curr.m_parent; p != null_v; p = all_nodes[p].m_parent) {
                        result[(result.size() - 1U) - i] = utils::to_nd(p, _params.size);
                        ++i;
                    }

                    std::cout << "Final path length: " << result.size() << "\n";

                    return result;
                }
                else { // SEARCH FOR SOLUTION...

                    // Expand successors if not already:
                    bool expanded = !curr.m_successors.empty() || curr.m_fScore == std::numeric_limits<scalar_t>::max();
                    if (!expanded) {

                        if (curr.m_depth == _params.memory_limit) {
                            curr.m_fScore = std::numeric_limits<scalar_t>::max();
                        }
                        else {

                            for (const auto& n_data : _params.maze.get_neighbours(curr.m_index)) {

                                if (const auto& n = solver_t::get_data(n_data, _params); n.active) {

                                    curr.m_successors.push_back(n.index);

                                    const auto g = curr.m_gScore + n.distance;
                                    const auto h = _params.h(n.coord, _params.end) * _params.weight;

                                    // Create or update successor node:
                                    auto child_search = all_nodes.find(n.index);
                                    if (child_search == all_nodes.end()) {

                                        // Attempt to create room for new node (if necessary):
                                        bool success = true;
                                        while (memory_usage(open, all_nodes) >= _params.memory_limit) {
                                            if (!remove_weakest(open, all_nodes, curr_idx)) {
                                                success = false;
                                                break;
                                            }
                                        }

                                        if (success) {
                                            all_nodes[n.index] = node(curr.m_depth + 1U, n.index, g, h, curr.m_index);
                                            open.emplace(n.index);
                                        }
                                    }
                                    else {

                                        // Update existing node and reinsert:
                                        auto& child_node = child_search->second;
                                        if (g < child_node.m_gScore) {

                                            open.erase(n.index);

                                            child_node.m_depth  = curr.m_depth + 1U;
                                            child_node.m_gScore = g;
                                            child_node.m_fScore = std::max(g + h, curr.m_fScore);
                                            child_node.m_parent = curr.m_index;

                                            open.emplace(n.index);
                                        }
                                    }
                                }
                            }
                        }
                    }

                    // Handle case where node has no successors (dead end):
                    if (curr.m_successors.empty()) {
                        curr.m_fScore = std::numeric_limits<scalar_t>::max();
                        backup_f_values(curr, all_nodes);
                    }
                    else if (!expanded && memory_usage(open, all_nodes) < _params.memory_limit) {
                        open.emplace(curr_idx);
                    }
                }
            }

            return std::vector<coord_t>{};
        }

        [[maybe_unused, nodiscard]] static auto invoke(const params_t& _params) {

            if (_params.memory_limit > 0U) {
                return solve_internal(_params);
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