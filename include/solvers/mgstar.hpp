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
     *
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

        static_assert(std::is_arithmetic_v<scalar_t>, "scalar_t must be an integral or floating point type.");
        static_assert(std::is_integral_v<index_t>, "index_t must be an integral type.");

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
                       _a.m_gScore >  _b.m_gScore :
                       _a.m_fScore >  _b.m_fScore;
            }
        };

        template <typename open_set_t, typename closed_set_t>
        [[maybe_unused, nodiscard]] HOT static constexpr auto solve_internal(open_set_t& _open, closed_set_t& _closed, size_t _capacity, const params_t& _params) {

            const auto s = utils::to_1d(_params.start, _params.size);
            const auto e = utils::to_1d(_params.end,   _params.size);

              _open.emplace_nosort(s, static_cast<scalar_t>(0), _params.h(_params.start, _params.end) * _params.weight);
            _closed.emplace(s);

            size_t dynamic_allocations(0U);
            size_t  closed_allocations(1U);

            // Define heuristic for quantifying memory usage:
            const auto memory_usage = [&_open, &closed_allocations, &dynamic_allocations]() ALWAYS_INLINE {
                return _open.size() + closed_allocations + dynamic_allocations;
            };

            stack<node*> expunct(_params.homogeneous_pmr);

            std::optional<node> final;

            // Main loop:
            while (LIKELY(!_open.empty())) {

                auto curr(std::move(_open.top()));
                _open.pop();

                if (curr.m_index != e) { // SEARCH FOR SOLUTION...

                    node* RESTRICT curr_ptr(nullptr);

                    if (!final.has_value() || curr.m_gScore < final->m_gScore) {

                        for (const auto& n_data : _params.maze.get_neighbours(curr.m_index)) {

                            if (const auto& n = solver_t::get_data(n_data, _params); n.active) {

                                // Check if node is not already visited:
                                if (!_closed.contains(n.index)) {

                                    bool full = memory_usage() >= _params.memory_limit;
                                    if (full) {

                                        if (!expunct.empty()) {
                                            full = false;

                                            _params.homogeneous_pmr->deallocate(std::move(expunct.top()), sizeof(node), alignof(node)); dynamic_allocations--;
                                            expunct.pop();
                                        }
                                        else if (!_open.empty()) {
                                            full = false;

                                            auto worst_itr = _open.begin(); // node with highest f-cost in _open.
                                            for (auto it = _open.begin(); it != _open.end(); ++it) {
                                                if (it->m_fScore > worst_itr->m_fScore) {
                                                    worst_itr = it;
                                                }
                                            }

                                            _closed.erase(worst_itr->m_index);
                                              _open.erase(worst_itr);
                                        }
                                    }

                                    if (!full) {
                                        solver_t::solver_utils::preallocate_emplace(_closed, n.index, _capacity, _params.maze.count());

                                        if (curr_ptr == nullptr) {
                                            curr_ptr = new (_params.homogeneous_pmr->allocate(sizeof(node), alignof(node))) node(std::move(curr)); dynamic_allocations++;
                                        }

                                        if constexpr (params_t::lazy_sorting::value) {
                                            _open.emplace_nosort(n.index, curr_ptr->m_gScore + n.distance, _params.h(n.coord, _params.end) * _params.weight, curr_ptr);
                                        }
                                        else {
                                            _open.emplace(n.index, curr_ptr->m_gScore + n.distance, _params.h(n.coord, _params.end) * _params.weight, curr_ptr);
                                        }
                                    }
                                    else {
                                        if (auto p = curr.m_parent; p != nullptr) {
                                            _closed.erase(curr.m_index);
                                            _open.emplace(*static_cast<node*>(p));
                                        }
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

                    if (!final.has_value() || curr.m_gScore < final->m_gScore) {
                        final.emplace(std::move(curr));
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

            return final.has_value() ?
                solver_t::solver_utils::rbacktrack(final.value(), _params.size, final.value().m_gScore) :
                std::vector<coord_t>{};
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

#endif //CHDR_MGSTAR_HPP