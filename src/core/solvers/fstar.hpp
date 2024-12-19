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

#include "../utils/heuristics.hpp"
#include "base/solver.hpp"
#include "mazes/graph.hpp"
#include "mazes/grid.hpp"
#include "solvers/base/unmanaged_node.hpp"
#include "types/coord.hpp"
#include "types/existence_set.hpp"
#include "types/heap.hpp"
#include "types/stable_forward_buf.hpp"
#include "utils/utils.hpp"

namespace chdr::solvers {

    template<size_t Kd, typename scalar_t, typename index_t, typename params_t>
    struct [[maybe_unused]] fstar final {

        friend struct solver<fstar, Kd, scalar_t, index_t, params_t>;

        static_assert(std::is_integral_v<scalar_t> || std::is_floating_point_v<scalar_t>, "scalar_t must be either an integral or floating point type.");
        static_assert(std::is_integral_v<index_t>, "index_t must be an integral type.");

    private:

        using solver_t = solver<fstar, Kd, scalar_t, index_t, params_t>;
        using  coord_t = coord<index_t, Kd>;

        struct node final : unmanaged_node<index_t> {

            scalar_t m_gScore;
            scalar_t m_fScore;

            /**
             * @brief Constructs an uninitialized ASNode.
             *
             * This constructor creates an ASNode with uninitialized members.
             */
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

        template <typename closed_set_t, typename buf_t>
        [[nodiscard]] static constexpr auto solve_internal(closed_set_t& _closed, buf_t& _buf, const size_t& _capacity, const params_t& _params) {

            const auto s = utils::to_1d(_params.start, _params.size);
            const auto e = utils::to_1d(_params.end,   _params.size);

            float min_threshold = _params.h(_params.start, _params.end) * _params.weight;

            std::vector<node> open;
            open.reserve(_capacity);

            std::vector<node> next;
            next.reserve(_capacity);

            open.emplace_back(s, static_cast<scalar_t>(0), _params.h(_params.start, _params.end) * _params.weight);

            // Main loop:
            while (!open.empty()) {

                auto next_threshold = std::numeric_limits<scalar_t>::max();

                for (const auto& curr : open) {

                    if (curr.m_index != e) { // SEARCH FOR SOLUTION...

                        _closed.allocate(curr.m_index, _capacity, _params.maze.count());
                        _closed.emplace (curr.m_index);

                        node* curr_ptr = nullptr;

                        for (const auto& n_data : _params.maze.get_neighbours(curr.m_index)) {

                            if (const auto& n = solver_t::get_data(n_data, _params); n.active) {

                                // Check if node is not already visited:
                                if (!_closed.contains(n.index)) {
                                     _closed.allocate(n.index, _capacity, _params.maze.count());
                                     _closed.emplace (n.index);

                                    const auto g = curr.m_gScore + n.distance;
                                    const auto f = g + (_params.h(n.coord, _params.end) * _params.weight);

                                    if (f <= min_threshold) {

                                        if (curr_ptr == nullptr) {
                                            curr_ptr = &_buf.emplace(std::move(curr));
                                        }

                                        const auto new_node = node(n.index, g, f, curr_ptr);

                                        const auto insertion_point = std::partition_point(next.begin(), next.end(), [&new_node](const node& _other) [[always_inline]] {
                                            return _other < new_node;
                                        });

                                        // Insert with tie-breaking rules applied:
                                        if (insertion_point == next.end()) {
                                            next.emplace_back(new_node);
                                        }
                                        else {
                                            next.insert(insertion_point, new_node);
                                        }
                                    }
                                    else {
                                        next_threshold = std::min(next_threshold, f);
                                    }
                                }
                            }
                        }
                    }
                    else { // SOLUTION REACHED ...
                        return curr.template backtrack<node>(_params.size, curr.m_gScore);
                    }
                }

                open = std::move(next);
                next.clear();

                min_threshold = next_threshold;
            }

            return std::vector<coord_t>{};
        }

        [[maybe_unused, nodiscard]] static constexpr auto execute(const params_t& _params) {

            const auto s = utils::to_1d(_params.start, _params.size);

            const auto capacity = solver_t::determine_capacity(_params);

            existence_set<low_memory_usage> closed({ s }, capacity);

            stable_forward_buf<node> buf;

            return solve_internal(closed, buf, capacity, _params);
        }
    };

} //chdr::solvers

#endif //CHDR_FRINGE_HPP