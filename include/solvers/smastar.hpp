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
#include <set>
#include <type_traits>
#include <vector>

#include "../utils/utils.hpp"
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

        using  coord_t = typename params_t:: coord_type;
        using  index_t = typename params_t:: index_type;
        using scalar_t = typename params_t::scalar_type;
        using solver_t = solver<smastar, params_t>;
        using weight_t = typename params_t::weight_type;

        static_assert(std::is_arithmetic_v<scalar_t>, "scalar_t must be an integral or floating point type.");
        static_assert(std::is_integral_v<index_t>, "index_t must be an integral type.");

        static constexpr auto null_v = std::numeric_limits< index_t>::max();
        static constexpr auto  inf_v = std::numeric_limits<scalar_t>::max();

        static constexpr auto Kd = std::tuple_size_v<std::decay_t<typename params_t::coord_type>>;

        struct node final {

             index_t m_depth;
             index_t m_index;
            scalar_t m_gScore;
            scalar_t m_fScore;
             index_t m_parent;

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
                m_parent    (_parent          ) {}

            ~node() = default;

            node           (const node&) = default;
            node& operator=(const node&) = default;

            [[nodiscard]] HOT constexpr node(node&& _other) noexcept = default;

#if __cplusplus > 202302L
            constexpr
#endif
            HOT node& operator=(node&& _other) noexcept = default;

            [[nodiscard]] HOT friend constexpr bool operator < (const node& a, const node& b) noexcept {
                return a.m_fScore == b.m_fScore ?
                       a.m_gScore >  b.m_gScore :
                       a.m_fScore <  b.m_fScore;
            }
        };

        template<typename open_set_t, typename all_nodes_t>
        static void remove_worst(open_set_t& _open, all_nodes_t& _all_nodes, const params_t& _params) {

            assert(!_open.empty() && "_open cannot be empty.");

            auto worst_node = std::prev(_open.end());
            backup_f_values(*worst_node, _all_nodes, _params);
            _all_nodes.erase(worst_node->m_index);
            _open.erase(worst_node);
        }

        template<typename all_nodes_t>
        static void backup_f_values(const node& _removed_node, all_nodes_t& _all_nodes, const params_t& _params) {

            if (auto p_index = _removed_node.m_parent; p_index != null_v) {

                for (auto p = _all_nodes.find(p_index); p != _all_nodes.end(); p_index = p->second.m_parent) {

                    bool has_children = false;
                    auto min_child_f = inf_v;

                    for (const auto& n_data : _params.maze.get_neighbours(p->second.m_index)) {

                        if (const auto& n = solver_t::get_data(n_data, _params); n.active) {

                            if (auto child_it = _all_nodes.find(n.index); child_it != _all_nodes.end()) {
                                has_children = true;
                                min_child_f = std::min(min_child_f, child_it->second.m_fScore);
                            }
                        }
                    }

                    // If there are children and updating the parent's f-value is necessary:
                    if (has_children && min_child_f > p->second.m_fScore) {
                        p->second.m_fScore = min_child_f;
                    }
                    else {
                        break;
                    }
                }
            }
        }

        template <typename open_set_t>
        [[nodiscard]] HOT static constexpr auto solve_internal(open_set_t& _open, const params_t& _params) {

            const auto s = utils::to_1d(_params.start, _params.size);
            const auto e = utils::to_1d(_params.end,   _params.size);

            std::unordered_map<index_t, node> all_nodes;

            _open.emplace(
                all_nodes[static_cast<index_t>(s)] = node(
                    static_cast< index_t>(0),
                    static_cast< index_t>(s),
                    static_cast<scalar_t>(0),
                    _params.h(_params.start, _params.end) * _params.weight
                )
            );

            while (!_open.empty()) {

                std::cout << all_nodes.size() << "\n";

                auto curr = _open.extract(_open.begin()).value();

                if (curr.m_index == e) { // SOLUTION FOUND...

                    std::vector<coord_t> result(curr.m_depth);

                    std::cout << result.size() << "\n";

                    size_t i = 0U;
                    for (auto p = curr.m_parent; p != null_v; p = all_nodes[p].m_parent, ++i) {
                        result[(result.size() - 1U) - i] = utils::to_nd(p, _params.size);
                    }

                    return result;
                }
                else { // SEARCH FOR SOLUTION...

                    size_t successor_count = 0U;

                    if (curr.m_fScore != inf_v) {

                        for (const auto& n_data : _params.maze.get_neighbours(curr.m_index)) {

                            if (const auto& n = solver_t::get_data(n_data, _params); n.active) {

                                const auto g = curr.m_gScore + n.distance;
                                const auto h = _params.h(n.coord, _params.end) * _params.weight;

                                auto child_search = all_nodes.find(n.index);
                                if (child_search == all_nodes.end()) {

                                    // Attempt to make room for the new node:
                                    if (!_open.empty() && all_nodes.size() >= _params.memory_limit) {
                                        remove_worst(_open, all_nodes, _params);
                                    }

                                    // assert(all_nodes.size() < _params.memory_limit && "SMA* memory overflow detected.");
                                    _open.emplace(all_nodes[n.index] = node(curr.m_depth + 1U, n.index, g, h, curr.m_index));

                                    successor_count++;
                                }
                                else if (g < child_search->second.m_gScore && child_search->second.m_fScore != inf_v) {

                                    _open.erase(child_search->second);
                                    child_search->second = node(curr.m_depth + 1U, n.index, g, h, curr.m_index);
                                    _open.emplace(child_search->second);

                                    successor_count++;
                                }
                            }
                        }
                    }

                    if (UNLIKELY(successor_count == 0U)) { // DEAD END...
                        curr.m_fScore = inf_v;
                        backup_f_values(curr, all_nodes, _params);
                        all_nodes[curr.m_index] = curr;
                    }
                }
            }

            return std::vector<coord_t>{};
        }

        [[maybe_unused, nodiscard]] static auto invoke(const params_t& _params) {

            if (_params.memory_limit > 0U) {

                std::pmr::multiset<node> _open(_params.homogeneous_pmr);

                return solve_internal(_open, _params);
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