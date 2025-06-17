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
#include "base/unmanaged_node.hpp"

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

        struct node final : unmanaged_node<index_t> {

            bool m_in_path;

             index_t m_depth;
            scalar_t m_gScore;
            scalar_t m_fScore;

            std::vector<index_t> m_successors;

            /**
             * @brief Constructs an uninitialized node.
             */
            // ReSharper disable once CppPossiblyUninitializedMember
            [[nodiscard]] HOT constexpr node() noexcept : unmanaged_node<index_t>() {} // NOLINT(*-pro-type-member-init, *-use-equals-default)

            [[nodiscard]] HOT constexpr node(index_t _depth, index_t _index, scalar_t _gScore, scalar_t _hScore, const unmanaged_node<index_t>* RESTRICT const _parent = nullptr) noexcept : unmanaged_node<index_t>(_index, _parent),
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

                _open.emplace(start_node);
            }

            // Main loop:
            while (LIKELY(!_open.empty())) {

                node curr(std::move(_open.extract(_open.begin()).value()));

                if (curr.m_index == e) { // SOLUTION REACHED...
                    return solver_t::solver_utils::rbacktrack(curr, _params.size, curr.m_depth);
                }
                else { // SEARCH FOR SOLUTION...

                    if (curr.m_successors.empty()) {

                        {
                            node* stored_curr = nullptr;

                            auto search = all_nodes.find(curr.m_index);
                            if (search != all_nodes.end()) {
                                stored_curr = &search->second;
                            }
                            else {
                                auto [it, inserted] = all_nodes.try_emplace(curr.m_index, std::move(curr));
                                stored_curr = &it->second;
                            }

                            curr = *stored_curr;
                        }

                        auto curr_coord = utils::to_nd(curr.m_index, _params.size);
                        std::cout << "stored_curr: " << curr.m_index << ": " << curr_coord[0U] << ", " << curr_coord[1U] << "\n";

                        // if (curr.m_index == 31) {
                        //     std::cout << ":think:\n";
                        //     std::cout << &_params.maze << "\n";
                        // }

                        for (const auto& n_data : _params.maze.get_neighbours(curr.m_index)) {

                            if (const auto& n = solver_t::get_data(n_data, _params); n.active) {

                                const auto g = curr.m_gScore + n.distance;
                                const auto h = _params.h(n.coord, _params.end) * _params.weight;

                                // Create or update node with best values.
                                auto child_search = all_nodes.find(n.index);
                                if (child_search == all_nodes.end()) {
                                    all_nodes[n.index] = node(curr.m_depth + 1U, n.index, g, h, &curr);
                                }
                                else {
                                    auto& child_node = child_search->second;
                                    if (!child_node.m_in_path && g < child_node.m_gScore) {

                                         child_node.m_depth  = curr.m_depth + 1U;
                                         child_node.m_gScore = g;
                                         child_node.m_fScore = g + h;
                                         child_node.m_parent = &curr;
                                    }
                                }

                                curr.m_successors.push_back(n.index);
                            }
                        }
                    }

                    // Select best successor from available:
                    node* best_successor = nullptr;
                    for (const auto& succ_idx : curr.m_successors) {
                        auto& succ = all_nodes[succ_idx];
                        if (!succ.m_in_path && (best_successor == nullptr || succ.m_fScore < best_successor->m_fScore)) {
                            best_successor = &succ;
                        }
                    }

                    // Add best successor to open set, or back-up the f-cost to the parent if not.
                    if (best_successor != nullptr) {
                        best_successor->m_in_path = true;
                        _open.emplace(*best_successor);
                    }
                    else {
                        scalar_t min_f = std::numeric_limits<scalar_t>::infinity();
                        for (const auto& succ_idx : curr.m_successors) {
                            auto& succ = all_nodes[succ_idx];
                            min_f = std::min(min_f, succ.m_fScore);
                        }
                        curr.m_fScore = std::max(curr.m_fScore, min_f);
                    }

                    // If memory is exhausted, purge worst node:
                    if (_open.size() > _params.memory_limit) {
                        auto worst_itr = std::prev(_open.end());
                        auto& worst = *const_cast<node*>(&*worst_itr);
                        worst.m_in_path = false;

                        _open.erase(worst_itr);
                    }
                }
            }

            return std::vector<coord_t>{};
        }

        [[maybe_unused, nodiscard]] static auto invoke(const params_t& _params) {

            assert(_params.memory_limit > 0U && "memory_limit must be greater than zero.");

            if (_params.memory_limit > 0U) {
                std::pmr::set<node> open(_params.heterogeneous_pmr);

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