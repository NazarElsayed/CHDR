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

#include "../types/containers/existence_set.hpp"
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

            [[nodiscard]] constexpr node(index_t _index, scalar_t _gScore, scalar_t _fScore, const unmanaged_node<index_t>* RESTRICT const _parent = nullptr) noexcept : unmanaged_node<index_t>(_index, _parent),
                m_gScore(_gScore),
                m_fScore(_fScore) {}

            [[nodiscard]] HOT friend constexpr bool operator < (const node& _a, const node& _b) noexcept {
                return _a.m_fScore == _b.m_fScore ?
                       _a.m_gScore >  _b.m_gScore :
                       _a.m_fScore >  _b.m_fScore;
            }
        };

        template <typename open_set_t, typename closed_set_t>
        [[nodiscard]] HOT static constexpr auto solve_internal(open_set_t& _open, open_set_t& _next, closed_set_t& _closed, size_t _capacity, const params_t& _params) {

            const auto s = utils::to_1d(_params.start, _params.size);
            const auto e = utils::to_1d(_params.end,   _params.size);

            auto max_threshold = _params.h(_params.start, _params.end) * _params.weight;

              _open.emplace_back(s, static_cast<scalar_t>(0), max_threshold);
            _closed.emplace(s);

            // Main loop:
            while (LIKELY(!_open.empty())) {

                auto next_threshold = std::numeric_limits<scalar_t>::max();

                for (auto& curr : _open) {

                    if (curr.m_index != e) { // SEARCH FOR SOLUTION...

                        node* RESTRICT curr_ptr(nullptr);

                        for (const auto& n_data : _params.maze.get_neighbours(curr.m_index)) {

                            if (const auto& n = solver_t::get_data(n_data, _params); n.active) {

                                const auto g = curr.m_gScore + n.distance;
                                const auto f = g + (_params.h(n.coord, _params.end) * _params.weight);

                                if (f <= max_threshold) {

                                    // Check if node is not already visited:
                                    if (!_closed.contains(n.index)) {

                                        solver_t::solver_utils::preallocate_emplace(_closed, n.index, _capacity, _params.maze.count());

                                        if (curr_ptr == nullptr) {
                                            curr_ptr = new (_params.monotonic_pmr->allocate(sizeof(node), alignof(node))) node(std::move(curr));
                                        }

                                        if constexpr (params_t::lazy_sorting::value) {
                                            _next.emplace_back(n.index, g, f, curr_ptr);
                                        }
                                        else {

                                            /* SORTED INSERTION */
                                            _next.insert(
                                                std::partition_point(
                                                    _next.begin(),
                                                    _next.end(),
                                                    [&f, &g](const node& _other) ALWAYS_INLINE {
                                                        return f == _other.m_fScore ?
                                                               g >  _other.m_gScore :
                                                               f >  _other.m_fScore;
                                                    }
                                                ),
                                                node (n.index, g, f, curr_ptr)
                                            );
                                        }
                                    }
                                }
                                else {
                                    next_threshold = utils::min(next_threshold, f);
                                    break;
                                }
                            }
                        }
                    }
                    else { // SOLUTION REACHED ...

                        _next   = std::move(open_set_t());
                        _closed = {};

                        return solver_t::solver_utils::rbacktrack(curr, _params.size, curr.m_gScore);
                    }
                }

                if (!_next.empty()) {
                    std::swap(_open, _next);
                    _next.clear();
                }

                max_threshold = next_threshold;
            }

            return std::vector<coord_t>{};
        }

        [[maybe_unused, nodiscard]] static auto execute(const params_t& _params) {

            const auto capacity = solver_t::solver_utils::determine_capacity(_params);

            existence_set closed(_params.monotonic_pmr);
            closed.reserve(capacity);

            std::pmr::vector<node> open(_params.polytonic_pmr);
            try {
                open.reserve(capacity / 8U);
            }
            catch (...) {} // NOLINT(*-empty-catch)

            std::pmr::vector<node> next(_params.polytonic_pmr);
            try {
                open.reserve(capacity / 8U);
            }
            catch (...) {} // NOLINT(*-empty-catch)

            return solve_internal(open, next, closed, capacity, _params);
        }
    };

} //chdr::solvers

#endif //CHDR_FRINGE_HPP