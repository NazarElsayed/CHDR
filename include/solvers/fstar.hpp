/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_FRINGE_HPP
#define CHDR_FRINGE_HPP

#include <cstddef>
#include <limits>
#include <vector>

#include "../types/append_only_allocator.hpp"
#include "../types/existence_set.hpp"
#include "../utils/utils.hpp"
#include "base/solver.hpp"
#include "base/unmanaged_node.hpp"

namespace chdr::solvers {

    template<typename params_t>
    struct [[maybe_unused]] fstar final {

        friend class solver<fstar, params_t>;

    private:

        using  index_t = typename params_t:: index_type;
        using scalar_t = typename params_t::scalar_type;
        using  coord_t = typename params_t:: coord_type;
        using solver_t = solver<fstar, params_t>;

        static_assert(std::is_arithmetic_v<scalar_t>, "scalar_t must be an integral or floating point type.");
        static_assert(std::numeric_limits<scalar_t>::is_specialized, "scalar_t must be a numeric type with defined numeric limits.");
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

            [[nodiscard]] constexpr node(const index_t& _index, const scalar_t& _gScore, const scalar_t& _fScore, const unmanaged_node<index_t>* RESTRICT const _parent = nullptr) noexcept : unmanaged_node<index_t>(_index, _parent),
                m_gScore(_gScore),
                m_fScore(_fScore) {}

            [[nodiscard]] friend constexpr bool operator < (const node& _a, const node& _b) noexcept {
                return _a.m_fScore == _b.m_fScore ?
                       _a.m_gScore >  _b.m_gScore :
                       _a.m_fScore >  _b.m_fScore;
            }
        };

        template <typename open_set_t, typename closed_set_t, typename alloc_t>
        [[nodiscard]] static constexpr auto solve_internal(open_set_t& _open, open_set_t& _next, closed_set_t& _closed, alloc_t& _alloc, const size_t& _capacity, const params_t& _params) {

            const auto s = utils::to_1d(_params.start, _params.size);
            const auto e = utils::to_1d(_params.end,   _params.size);

            auto min_threshold = _params.h(_params.start, _params.end) * _params.weight;

               _open.emplace_back(s, static_cast<scalar_t>(0), min_threshold);
             _closed.emplace(s);

            // Main loop:
            while (!_open.empty()) {

                auto next_threshold = std::numeric_limits<scalar_t>::max();

                for (const auto& curr : _open) {

                    if (curr.m_index != e) { // SEARCH FOR SOLUTION...

                        node* RESTRICT curr_ptr = nullptr;

                        for (const auto& n_data : _params.maze.get_neighbours(curr.m_index)) {

                            if (const auto& n = solver_t::get_data(n_data, _params); n.active) {

                                // Check if node is not already visited:
                                if (!_closed.contains(n.index)) {
                                    solver_utils::preallocate_emplace(_closed, n.index, _capacity, _params.maze.count());

                                    const auto g = curr.m_gScore + n.distance;
                                    const auto f = g + (_params.h(n.coord, _params.end) * _params.weight);

                                    if (f <= min_threshold) {

                                        if (curr_ptr == nullptr) {
                                            _alloc.construct(curr_ptr = _alloc.allocate(1U), std::move(curr)); // Note: 'current' is now moved!
                                        }

                                        /* SORTED INSERTION */

                                        const auto new_node = node(n.index, g, f, curr_ptr);

                                        _next.insert(
                                            std::partition_point(
                                                _next.begin(),
                                                _next.end(),
                                                [&new_node](const node& _other) ALWAYS_INLINE { return _other < new_node; }
                                            ),
                                            new_node
                                        );
                                    }
                                    else {
                                        next_threshold = utils::min(next_threshold, f);
                                    }
                                }
                            }
                        }
                    }
                    else { // SOLUTION REACHED ...

                        _next   = {};
                        _closed = {};

                        const auto result = solver_utils::rbacktrack(curr, _params.size, curr.m_gScore);

                        _open = {};

                        return result;
                    }
                }

                std::swap(_open, _next);
                _next.clear();

                min_threshold = next_threshold;
            }

            _open   = {};
            _next   = {};
            _closed = {};

            return std::vector<coord_t>{};
        }

        [[maybe_unused, nodiscard]] static auto execute(const params_t& _params) {

            const auto capacity = solver_t::determine_capacity(_params);

            existence_set closed;
            closed.reserve(capacity);

            std::vector<node> open;
            try {
                open.reserve(capacity / 8U);
            }
            catch ([[maybe_unused]] const std::exception& e) {} // NOLINT(*-empty-catch)

            std::vector<node> next;
            try {
                open.reserve(capacity / 8U);
            }
            catch ([[maybe_unused]] const std::exception& e) {} // NOLINT(*-empty-catch)

            forward_allocator<node> alloc;

            return solve_internal(open, next, closed, alloc, capacity, _params);
        }
    };

} //chdr::solvers

#endif //CHDR_FRINGE_HPP