/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_GSTAR_HPP
#define CHDR_GSTAR_HPP

#include <cstddef>

#include "base/solver.hpp"
#include "solvers/base/managed_node.hpp"
#include "types/coord.hpp"
#include "types/existence_set.hpp"
#include "types/heap.hpp"
#include "utils/intrinsics.hpp"
#include "utils/utils.hpp"

namespace chdr::solvers {

    template<size_t Kd, typename scalar_t, typename index_t, typename params_t>
    struct [[maybe_unused]] gstar final {

        friend struct solver<gstar, Kd, scalar_t, index_t, params_t>;

        static_assert(std::is_integral_v<scalar_t> || std::is_floating_point_v<scalar_t>, "scalar_t must be either an integral or floating point type");
        static_assert(std::is_integral_v<index_t>, "index_t must be an integral type.");

    private:

        using solver_t = solver<gstar, Kd, scalar_t, index_t, params_t>;
        using  coord_t = coord<index_t, Kd>;

        struct node final : managed_node<index_t> {

            scalar_t m_gScore;
            scalar_t m_fScore;

            /**
             * @brief Constructs an uninitialized ASNode.
             *
             * This constructor creates an ASNode with uninitialized members.
             */
            [[nodiscard]] constexpr node() noexcept : managed_node<index_t>() {}; // NOLINT(*-pro-type-member-init, *-use-equals-default)

            [[nodiscard]] constexpr node(const index_t& _index, const scalar_t& _gScore, const scalar_t& _hScore, node* RESTRICT const _parent = nullptr) noexcept : managed_node<index_t>(_index, _parent),
                m_gScore     (_gScore          ),
                m_fScore     (_gScore + _hScore) {}

            [[nodiscard]] friend constexpr bool operator < (const node& _a, const node& _b) noexcept {
                return _a.m_fScore == _b.m_fScore ?
                       _a.m_gScore >  _b.m_gScore :
                       _a.m_fScore >  _b.m_fScore;
            }
        };

        template <typename open_set_t, typename closed_set_t>
        [[maybe_unused, nodiscard]] static constexpr auto solve_internal(open_set_t& _open, closed_set_t& _closed, const size_t& _capacity, const params_t& _params) {

            const auto s = utils::to_1d(_params.start, _params.size);
            const auto e = utils::to_1d(_params.end,   _params.size);

            _open.emplace(s, static_cast<scalar_t>(0), _params.h(_params.start, _params.end));

            // Main loop:
            while (!_open.empty()) {

                auto curr(std::move(_open.top()));
                _open.pop();

                if (curr.m_index != e) { // SEARCH FOR SOLUTION...

                    _closed.allocate(curr.m_index, _capacity, _params.maze.count());
                    _closed.emplace (curr.m_index);

                    node* RESTRICT curr_ptr = nullptr;

                    for (const auto& n_data : _params.maze.get_neighbours(curr.m_index)) {

                        if (const auto& n = solver_t::get_data(n_data, _params); n.active) {

                            // Check if node is not already visited:
                            if (!_closed.contains(n.index)) {
                                 _closed.allocate(n.index, _capacity, _params.maze.count());
                                 _closed.emplace (n.index);

                                if (curr_ptr == nullptr) {
                                    curr_ptr = new node(curr);
                                }

                                _open.emplace(n.index, curr.m_gScore + n.distance, _params.h(n.coord, _params.end) * _params.weight, curr_ptr);
                            }
                        }
                    }
                }
                else { // SOLUTION REACHED ...

                    // Free data which is no longer relevant:

                    if constexpr (utils::has_method_clear        <  open_set_t>::value) {   _open.clear();         }
                    if constexpr (utils::has_method_shrink_to_fit<  open_set_t>::value) {   _open.shrink_to_fit(); }
                    if constexpr (utils::has_method_clear        <closed_set_t>::value) { _closed.clear();         }
                    if constexpr (utils::has_method_shrink_to_fit<closed_set_t>::value) { _closed.shrink_to_fit(); }

                    return curr.template backtrack<node>(_params.size, curr.m_gScore);
                }
            }

            return std::vector<coord_t>{};
        }

        [[maybe_unused, nodiscard]] static constexpr auto execute(const params_t& _params) {

            const auto s = utils::to_1d(_params.start, _params.size);

            const auto capacity = solver_t::determine_capacity(_params);

            existence_set closed({ s }, capacity);

            heap<node> open;
            open.reserve(capacity / 8U);

            return solve_internal(open, closed, capacity, _params);
        }
    };

} //chdr::solvers

#endif //CHDR_GSTAR_HPP