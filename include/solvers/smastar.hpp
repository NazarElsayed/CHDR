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

                bool m_enqueued;
             index_t m_depth;
            scalar_t m_gScore;
            scalar_t m_fScore;

            std::vector<node*> m_parents;
            std::vector<node*> m_successors;

            /**
             * @brief Constructs an uninitialized node.
             */
            // ReSharper disable once CppPossiblyUninitializedMember
            [[nodiscard]] HOT constexpr node() noexcept : unmanaged_node<index_t>() {} // NOLINT(*-pro-type-member-init, *-use-equals-default)

            [[nodiscard]] HOT constexpr node(bool _enqueued, index_t _depth, index_t _index, scalar_t _gScore, scalar_t _hScore, const unmanaged_node<index_t>* RESTRICT const _parent = nullptr) noexcept : unmanaged_node<index_t>(_index, _parent),
                m_enqueued  (_enqueued        ),
                m_depth     (_depth           ),
                m_gScore    (_gScore          ),
                m_fScore    (_gScore + _hScore),
                m_parents   (),
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

            std::unordered_map<index_t, node> generated;

            {
                node start(true, static_cast<index_t>(0), static_cast<index_t>(s), static_cast<scalar_t>(0), _params.h(_params.start, _params.end) * _params.weight);
                _open.emplace(start);
                generated.emplace(s, start);
            }

            // Main loop:
            while (LIKELY(!_open.empty())) {

                node curr(std::move(_open.extract(_open.begin()).value()));

                std::cout << "Iteration: " << curr.m_index << "\n";

                if (curr.m_index == e) { // SOLUTION REACHED ...

                    if constexpr (std::is_same_v<std::decay_t<decltype(_open)>, heap<node>>) {
                        _open.wipe();
                    }
                    else {
                        _open = open_set_t{};
                    }

                    return solver_t::solver_utils::rbacktrack(curr, _params.size, curr.m_gScore);
                }
                else { // SEARCH FOR SOLUTION...

                    // Expand (generate successors of curr):
                    if (curr.m_successors.empty()) {
                        for (const auto& n_data : _params.maze.get_neighbours(curr.m_index)) {
                            if (const auto& n = solver_t::get_data(n_data, _params); n.active) {
                                node* n_ptr(nullptr);

                                auto search = generated.find(n.index);
                                if (search == generated.end()) {
                                    auto [it, success] = generated.try_emplace(
                                        n.index,
                                        false,
                                        curr.m_depth + 1U,
                                        n.index,
                                        curr.m_gScore + n.distance,
                                        _params.h(n.coord, _params.end) * _params.weight,
                                        &curr
                                    );

                                    n_ptr = &(it->second);
                                    n_ptr->m_parents.push_back(&curr);
                                }
                                else {
                                    n_ptr = &(search->second);
                                    if (std::find(n_ptr->m_parents.begin(), n_ptr->m_parents.end(), &curr) == n_ptr->m_parents.end()) {
                                        n_ptr->m_parents.push_back(&curr);
                                    }
                                }

                                curr.m_successors.push_back(n_ptr);
                            }
                        }
                    }

                    node* succ(nullptr);
                    for (auto* successor : curr.m_successors) {
                        if (!successor->m_enqueued) {
                            succ = successor;
                            break;
                        }
                    }

                    if (succ != nullptr) {
                        if (succ->m_index != e && succ->m_depth == _params.memory_limit) {
                            succ->m_fScore = std::numeric_limits<scalar_t>::infinity();
                        }
                        else {
                            const auto sCoord = utils::to_nd(succ->m_index, _params.size);
                            succ->m_fScore    = std::max(succ->m_fScore, succ->m_gScore + _params.h(sCoord, _params.end) * _params.weight);
                        }
                    }
                    else {
                        // Update current node's f-cost to minimum of successor f-costs.
                        if (!curr.m_successors.empty()) {
                            auto min_successor_f = std::numeric_limits<scalar_t>::infinity();
                            for (const auto* successor : curr.m_successors) {
                                min_successor_f = std::min(min_successor_f, successor->m_fScore);
                            }
                            curr.m_fScore = std::max(curr.m_fScore, min_successor_f);
                        }

                        // Propagate cost updates to parent nodes if needed.
                        for (auto* parent : curr.m_parents) {
                            // Calculate what the parent's f-cost should be based on current node's updated cost.
                            scalar_t new_parent_f = std::numeric_limits<scalar_t>::infinity();

                            // Find minimum f-cost among all of parent's successors:
                            for (const auto* successor : parent->m_successors) {
                                new_parent_f = std::min(new_parent_f, successor->m_fScore);
                            }

                            // Update parent's f-cost if the new minimum is higher than current:
                            if (new_parent_f > parent->m_fScore) {
                                // Find the parent in the open set:
                                auto parent_it = _open.find(*parent);
                                if (parent_it != _open.end()) {
                                    // Remove from set, and re-insert with updated cost.
                                    _open.erase(parent_it);
                                    parent->m_fScore = new_parent_f;
                                    _open.emplace(*parent);
                                }
                                else {
                                    // Parent not in open set, just update the cost.
                                    parent->m_fScore = new_parent_f;
                                }
                            }
                        }
                    }

                    bool all_successors_in_queue = true;
                    for (auto* successor : curr.m_successors) {
                        if (!successor->m_enqueued) {
                            all_successors_in_queue = false;
                            break;
                        }
                    }
                    if (all_successors_in_queue) {
                        curr.m_enqueued = false;
                    }

                    // memory management
                    if (_open.size() >= _params.memory_limit) {

                        // Find shallowest node with highest f-cost.
                        // Remove it and update parents:

                        const auto& bad      = *std::prev(_open.end());
                        const auto& bad_node = bad;

                        for (auto* parent : bad_node.m_parents) {
                            parent->m_successors.erase(
                                std::remove_if(
                                    parent->m_successors.begin(),
                                    parent->m_successors.end(),
                                    [&bad_node](const node* n) { return n == &bad_node; }
                                ),
                                parent->m_successors.end()
                            );

                            if (parent->m_successors.empty()) {
                                _open.emplace(*parent);
                            }
                        }

                        _open.erase(bad);
                    }

                    if (succ != nullptr && !succ->m_enqueued) {
                        succ->m_enqueued = true;
                        _open.emplace(*succ);
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