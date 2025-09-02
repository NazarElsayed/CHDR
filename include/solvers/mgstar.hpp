/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_MGSTAR_HPP
#define CHDR_MGSTAR_HPP

/**
 * @file mgstar.hpp
 */

#include <cstddef>
#include <set>
#include <optional>

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
     * @ingroup SingleTargetMemoryBounded Memory-Bounded
     */

    /**
     * @struct mgstar
     * @brief Memory-Bounded Graveyard search algorithm.
     * @details MG* (Eriksson, L., 2025) is a heuristic-informed graph traversal and pathfinding algorithm for
     *          "single-source, single-target" (SSST) pathfinding problems.
     *          MG* maintains the number of expanded nodes in memory beneath an arbitrary limit, which it enforces
     *          through temporarily abandoning the worst-case search nodes to prioritise more promising candidates.
     *          Unlike the SMA* algorithm, MG* ensures an optimal solution by guaranteeing that prematurely discarded
     *          paths are explored fully before a final solution is reached.
     *
     * Advantages:
     * - Generally faster than both SMA* and OSMA*.
     * - Improved search range and memory efficiency over SMA*.
     * - Guarantees the optimal solution for the given memory limit.
     * - Able to find solutions to search problems in memory-constrained contexts.
     * - Able to modulate between a breadth-first and a best-first approach.
     * - Does not need a pre-pass, although performance can improve if the search space is pruned first.
     * - High performance in bounded (finite) search scenarios.
     *
     * Limitations:
     * - Slower than A*.
     * - Inefficient or complex search heuristics can reduce performance.
     * - Poor performance when searches lack solutions.
     *
     * References:
     * - Eriksson, L., 2025. MG*: An Improved Algorithm for Guaranteed Optimal-Cost Memory-Bounded Graph Search. Master’s. University of Exeter, Exeter, UK.
     *
     * @see smastar
     * @see osmastar
     * @tparam params_t Type containing the search parameters.
     */
    template<typename params_t>
    struct [[maybe_unused]] mgstar final {

        friend class solver<mgstar, params_t>;

    private:

        using  index_t = typename params_t:: index_type;
        using scalar_t = typename params_t::scalar_type;
        using  coord_t = typename params_t:: coord_type;
        using solver_t = solver<mgstar, params_t>;

        struct node final : managed_node<index_t, node> {

            scalar_t m_gScore;
            scalar_t m_fScore;

            /**
             * @brief Constructs an uninitialized node.
             */
            // ReSharper disable once CppPossiblyUninitializedMember
            [[nodiscard]] constexpr node() noexcept : managed_node<index_t, node>() {} // NOLINT(*-pro-type-member-init, *-use-equals-default)

            [[nodiscard]] constexpr node(index_t _index, scalar_t _gScore, scalar_t _hScore, node* RESTRICT const _parent = nullptr) noexcept : managed_node<index_t, node>(_index, _parent),
                m_gScore(_gScore          ),
                m_fScore(_gScore + _hScore) {}

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
                       _a.m_gScore <  _b.m_gScore :
                       _a.m_fScore <  _b.m_fScore;
            }
        };

        static constexpr auto rbacktrack(const node& _node, const coord_t& _size, size_t _depth) {

            std::vector<coord_t> result(_depth);

            const auto* RESTRICT t = &_node;
            for (size_t i = 0U; i != _depth; ++i) {
                result[(result.size() - 1U) - i] = utils::to_nd(t->m_index, _size);
                t = static_cast<const node*>(t->m_parent);
            }

            return result;
        }

        static constexpr auto rbacktrack_noreverse(const node& _node, const coord_t& _size, size_t _depth) {

            std::vector<coord_t> result(_depth);

            const auto* RESTRICT t = &_node;
            for (size_t i = 0U; i != _depth; ++i) {
                result[i] = utils::to_nd(t->m_index, _size);
                t = static_cast<const node*>(t->m_parent);
            }

            return result;
        }

        template <typename closed_set_t>
        HOT static void bitwise_regression(const managed_node<index_t, node>* _parent, closed_set_t& _closed) {

            if (_parent != nullptr) {
                for (auto* p = _parent; p->m_parent != nullptr && _closed.contains(p->m_index); p = p->m_parent) {
                    _closed.erase(p->m_index);
                }
            }
        }

        template <typename open_set_t, typename closed_set_t, typename expunct_set_t>
        [[nodiscard]] HOT static auto desaturate(open_set_t& _open, closed_set_t& _closed, expunct_set_t& _expunct, size_t& _dynamic_allocations, const params_t& _params) {

            bool result = false;
            if (!_expunct.empty()) { // LOSSLESS:

                _params.homogeneous_pmr->deallocate(std::move(_expunct.top()), sizeof(node), alignof(node)); _dynamic_allocations--;
                _expunct.pop();
            }
            else if (!_open.empty()) { // LOSSY:

                auto worst_node = std::prev(_open.end());
                bitwise_regression(worst_node->m_parent, _closed);
                _closed.erase(worst_node->m_index);
                  _open.erase(worst_node);
            }
            else {
                result = true;
            }

            return result;
        }

        template <typename open_set_t, typename closed_set_t>
        [[maybe_unused, nodiscard]] HOT static constexpr auto solve_internal(open_set_t& _open, closed_set_t& _closed, size_t _capacity, const params_t& _params) {

            constexpr bool optimising = true;

            const auto&   end = params_t::reverse_equivalence::value ? _params.start : _params.end;
            const auto& start = params_t::reverse_equivalence::value ? _params.end   : _params.start;

            const auto s = utils::to_1d(start, _params.size);
            const auto e = utils::to_1d(end,   _params.size);

              _open.emplace(s, static_cast<scalar_t>(0), _params.h(start, end) * _params.weight);
            _closed.emplace(s);

            size_t dynamic_allocations(0U);
            size_t  closed_allocations(1U);

            // Define heuristic for quantifying memory usage:
            const auto memory_usage = [&_open, &closed_allocations, &dynamic_allocations]() ALWAYS_INLINE {
                return _open.size() + closed_allocations + dynamic_allocations;
            };

            stack<node*> expunct(_params.heterogeneous_pmr);

            std::optional<node> best_solution;

            // Main loop:
            while (LIKELY(!_open.empty())) {

                auto curr(_open.extract(_open.begin()).value());

                if (curr.m_index != e) { // SEARCH FOR SOLUTION...

                    node* RESTRICT curr_ptr(nullptr);

                    if (!best_solution.has_value() || curr.m_gScore < best_solution->m_gScore) {

                        for (const auto& n_data : _params.maze.get_neighbours(curr.m_index)) {

                            if (const auto& n = solver_t::get_data(n_data, _params); n.active) {

                                // Check if node is not already visited:
                                if (!_closed.contains(n.index)) {

                                    // Attempt to resolve the issue of memory saturation.
                                    bool full = memory_usage() >= _params.memory_limit;
                                    if (full) {
                                        full = desaturate(_open, _closed, expunct, dynamic_allocations, _params);
                                    }

                                    if (!full) {
                                        solver_t::solver_utils::preallocate_emplace(_closed, n.index, _capacity, _params.maze.count());

                                        if (curr_ptr == nullptr) {
                                            curr_ptr = new (_params.homogeneous_pmr->allocate(sizeof(node), alignof(node))) node(std::move(curr)); dynamic_allocations++;
                                        }

                                        _open.emplace(n.index, curr_ptr->m_gScore + n.distance, _params.h(n.coord, end) * _params.weight, curr_ptr);
                                    }
                                    else {
                                        // Memory saturated. Backup losslessly...
                                        bitwise_regression(curr.m_parent, _closed);
                                        break;
                                    }
                                }
                            }
                        }
                    }

                    if (curr_ptr == nullptr) {

                        if (auto* p = curr.forget_one(); p != nullptr) {
                            expunct.emplace(static_cast<node*>(p));
                        }
                    }
                }
                else { // SOLUTION REACHED...

                    if (!best_solution.has_value() || curr.m_gScore < best_solution->m_gScore) {
                        best_solution.emplace(std::move(curr));

                        if constexpr (optimising) { // Optimising mechanism:

                            bool full = memory_usage() >= _params.memory_limit;

                            // ReSharper disable once CppUsingResultOfAssignmentAsCondition
                            if (full && ((full = !_open.empty()))) {
                                _open.clear();
                            }

                            if (!full) {
                                _open.emplace(s, static_cast<scalar_t>(0), _params.h(start, end) * _params.weight);
                            }
                        }
                    }
                }
            }

            if constexpr (std::is_same_v<std::decay_t<decltype(_open)>, heap<node>>) {
                _open.wipe();
            }
            else {
                _open = open_set_t{};
            }
            _closed = closed_set_t{};

            if (best_solution.has_value()) {

                if constexpr (params_t::reverse_equivalence::value) {
                    return solver_t::solver_utils::rbacktrack(best_solution.value(), _params.size, best_solution->m_gScore);
                }
                else {
                    return solver_t::solver_utils::rbacktrack_noreverse(best_solution.value(), _params.size, best_solution->m_gScore);
                }
            }
            else {
                return std::vector<coord_t>{};
            }
        }

        [[maybe_unused, nodiscard]] static auto invoke(const params_t& _params) {

            const auto capacity = solver_t::solver_utils::determine_capacity(_params);

            existence_set closed(_params.monotonic_pmr);
            closed.reserve(capacity);

            std::pmr::multiset<node> open(_params.heterogeneous_pmr);

            return solve_internal(open, closed, capacity, _params);
        }
    };

    /**
     * @}
     * @}
     * @}
     */

} //chdr::solvers

#endif //CHDR_MGSTAR_HPP