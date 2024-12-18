/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_BSTAR_HPP
#define CHDR_BSTAR_HPP

#include "mazes/graph.hpp"
#include "mazes/grid.hpp"
#include "solvers/base/unmanaged_node.hpp"
#include "types/existence_set.hpp"
#include "types/heap.hpp"
#include "types/stable_forward_buf.hpp"
#include "utils/utils.hpp"

namespace chdr::solvers {

    template<size_t Kd, typename scalar_t, typename index_t, typename params_t>
    struct [[maybe_unused]] bstar final {

        friend struct solver<bstar, Kd, scalar_t, index_t, params_t>;

        static_assert(std::is_integral_v<scalar_t> || std::is_floating_point_v<scalar_t>, "scalar_t must be either an integral or floating point type.");
        static_assert(std::is_integral_v<index_t>, "index_t must be an integral type.");

    private:

        using solver_t = solver<bstar, Kd, scalar_t, index_t, params_t>;
        using  coord_t = coord<index_t, Kd>;

        struct node final : unmanaged_node<index_t> {

            scalar_t m_hScore;

            /**
             * @brief Constructs an uninitialized BSNode.
             *
             * This constructor creates an BSNode with uninitialized members.
             */
            [[nodiscard]] constexpr node() noexcept {} // NOLINT(*-pro-type-member-init, *-use-equals-default)

            [[nodiscard]] constexpr node(const index_t& _index, const scalar_t& _hScore, const unmanaged_node<index_t>* RESTRICT const _parent = nullptr) noexcept : unmanaged_node<index_t>(_index, _parent),
                m_hScore(_hScore) {}

            [[nodiscard]] friend constexpr bool operator < (const node& _a, const node& _b) noexcept {
                return _a.m_hScore > _b.m_hScore;
            }
        };

        template <typename open_set_t, typename closed_set_t, typename buf_t>
        [[nodiscard]] static constexpr auto solve_internal(open_set_t& _open, closed_set_t& _closed, buf_t& _buf, const size_t& _capacity, const params_t& _params) {

            const auto s = utils::to_1d(_params.start, _params.size);
            const auto e = utils::to_1d(_params.end,   _params.size);

            _open.emplace(s, _params.h(_params.start, _params.end));

            // Main loop:
            while (!_open.empty()) {

                auto curr(std::move(_open.top()));
                _open.pop();

                if (curr.m_index != e) { // SEARCH FOR SOLUTION...

                    _closed.allocate(curr.m_index, _capacity, _params.maze.count());
                    _closed.emplace (curr.m_index);

                    for (const auto& n_data : _params.maze.get_neighbours(curr.m_index)) {

                        if (const auto& n = solver_t::get_data(n_data, _params); n.active) {

                            // Check if node is not already visited:
                            if (!_closed.contains(n.index)) {
                                 _closed.allocate(n.index, _capacity, _params.maze.count());
                                 _closed.emplace (n.index);

                                _open.emplace(n.index, _params.h(n.coord, _params.end), &_buf.emplace(std::move(curr)));
                            }
                        }
                    }
                }
                else { // SOLUTION REACHED ...
                    return curr.template backtrack<node>(_params.size, static_cast<size_t>(_params.h(_params.start, _params.end)));
                }
            }

            return std::vector<coord_t>{};
        }

        [[maybe_unused, nodiscard]] static constexpr auto execute(const params_t& _params) {

            const auto s = utils::to_1d(_params.start, _params.size);

            const auto capacity = solver_t::determine_capacity(_params);

            existence_set<low_memory_usage> closed({ s }, capacity);

            heap<node> open;
            open.reserve(capacity / 8U);

            stable_forward_buf<node> buf;

            return solve_internal(open, closed, buf, capacity, _params);
        }
    };

} //chdr::solvers

#endif //CHDR_BSTAR_HPP