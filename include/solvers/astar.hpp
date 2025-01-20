/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_ASTAR_HPP
#define CHDR_ASTAR_HPP

#include <cstddef>
#include <vector>

#include "../types/containers/existence_set.hpp"
#include "../types/containers/heap.hpp"
#include "../utils/utils.hpp"
#include "base/solver.hpp"
#include "base/unmanaged_node.hpp"

namespace chdr::solvers {

    template<typename params_t>
    struct [[maybe_unused]] astar final {

        friend class solver<astar, params_t>;

    private:

        using  index_t = typename params_t:: index_type;
        using scalar_t = typename params_t::scalar_type;
        using  coord_t = typename params_t:: coord_type;
        using solver_t = solver<astar, params_t>;

        static_assert(std::is_arithmetic_v<scalar_t>, "scalar_t must be an integral or floating point type.");
        static_assert(std::is_integral_v<index_t>, "index_t must be an integral type.");

        struct node final : unmanaged_node<index_t> {

            scalar_t m_gScore;
            scalar_t m_fScore;

            /**
             * @brief Constructs an uninitialized node.
             *
             * This constructor creates a node with uninitialized members.
             */
            // ReSharper disable once CppPossiblyUninitializedMember
            [[nodiscard]] constexpr node() noexcept : unmanaged_node<index_t>() {} // NOLINT(*-pro-type-member-init, *-use-equals-default)

            [[nodiscard]] constexpr node(const index_t& _index, const scalar_t& _gScore, const scalar_t& _hScore, const unmanaged_node<index_t>* RESTRICT const _parent = nullptr) noexcept : unmanaged_node<index_t>(_index, _parent),
                m_gScore(_gScore),
                m_fScore(_gScore + _hScore) {}

            [[nodiscard]] friend constexpr bool operator < (const node& _a, const node& _b) noexcept {
                return _a.m_fScore == _b.m_fScore ?
                       _a.m_gScore >  _b.m_gScore :
                       _a.m_fScore >  _b.m_fScore;
            }
        };

        template <typename open_set_t, typename closed_set_t>
        [[nodiscard]] static constexpr auto solve_internal(open_set_t& _open, closed_set_t& _closed, const size_t& _capacity, const params_t& _params) {

            const auto s = utils::to_1d(_params.start, _params.size);
            const auto e = utils::to_1d(_params.end,   _params.size);

              _open.emplace_nosort(s, static_cast<scalar_t>(0), _params.h(_params.start, _params.end) * _params.weight);
            _closed.emplace(s);

            // Main loop:
            while (!_open.empty()) {

                auto curr(std::move(_open.top()));
                _open.pop();

                if (curr.m_index != e) { // SEARCH FOR SOLUTION...

                    node* RESTRICT curr_ptr(nullptr);

                    for (const auto& n_data : _params.maze.get_neighbours(curr.m_index)) {

                        if (const auto& n = solver_t::get_data(n_data, _params); n.active) {

                            // Check if node is not already visited:
                            if (!_closed.contains(n.index)) {
                                solver_utils::preallocate_emplace(_closed, n.index, _capacity, _params.maze.count());

                                if (curr_ptr == nullptr) {
                                    curr_ptr = new (_params.monotonic_pmr->allocate(sizeof(node), alignof(node))) node(std::move(curr));
                                }

                                if constexpr (params_t::lazy_sorting::value) {
                                    _open.emplace_nosort(n.index, curr_ptr->m_gScore + n.distance, _params.h(n.coord, _params.end) * _params.weight, curr_ptr);
                                }
                                else {
                                    _open.emplace(n.index, curr_ptr->m_gScore + n.distance, _params.h(n.coord, _params.end) * _params.weight, curr_ptr);
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
                        _open = {};
                    }
                    _closed = {};

                    return solver_utils::rbacktrack(curr, _params.size, curr.m_gScore);
                }
            }

            return std::vector<coord_t>{};
        }

        [[maybe_unused, nodiscard]] static auto execute(const params_t& _params) {

            const auto capacity = solver_t::determine_capacity(_params);

            existence_set closed(_params.monotonic_pmr);
            closed.reserve(capacity);

            heap<node> open(_params.polytonic_pmr);
            try {
                open.reserve(capacity / 8U);
            }
            catch (...) {} // NOLINT(*-empty-catch)

            return solve_internal(open, closed, capacity, _params);
        }
    };

} //chdr::solvers

#endif //CHDR_ASTAR_HPP