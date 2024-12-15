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

#include "../utils/heuristics.hpp"
#include "base/solver.hpp"
#include "mazes/graph.hpp"
#include "mazes/grid.hpp"
#include "solvers/base/unmanaged_node.hpp"
#include "types/coord.hpp"
#include "types/existence_set.hpp"
#include "types/heap.hpp"
#include "types/linear_priority_queue.hpp"
#include "types/stable_forward_buf.hpp"
#include "types/stack_allocator.hpp"
#include "utils/utils.hpp"

namespace chdr::solvers {

    template<size_t Kd, typename scalar_t, typename index_t, typename params_t>
    struct [[maybe_unused]] astar final {

        friend struct solver<astar, Kd, scalar_t, index_t, params_t>;

        static_assert(std::is_integral_v<scalar_t> || std::is_floating_point_v<scalar_t>, "scalar_t must be either an integral or floating point type.");
        static_assert(std::is_integral_v<index_t>, "index_t must be an integral type.");

    private:

        using solver_t = solver<astar, Kd, scalar_t, index_t, params_t>;
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

            [[nodiscard]] constexpr node(const index_t& _index, const scalar_t& _gScore, const scalar_t& _hScore, const unmanaged_node<index_t>* RESTRICT const _parent = nullptr) noexcept : unmanaged_node<index_t>(_index, _parent),
                m_gScore(_gScore),
                m_fScore(_gScore + _hScore) {}

            [[nodiscard]] friend constexpr bool operator < (const node& _a, const node& _b) noexcept {
                return _a.m_fScore == _b.m_fScore ?
                       _a.m_gScore >  _b.m_gScore :
                       _a.m_fScore >  _b.m_fScore;
            }
        };

        template <typename open_set_t>
        [[maybe_unused, nodiscard]] static constexpr auto solve(const params_t& _params) {

            std::vector<coord_t> result;

            const auto s = utils::to_1d(_params.start, _params.size);
            const auto e = utils::to_1d(_params.end,   _params.size);

            // Create closed set:
            const auto capacity = std::max(_params.capacity, std::max(s, e));
            existence_set<low_memory_usage> closed({ s }, capacity);

            // Create open set:
            open_set_t open(capacity / 8U);
            open.emplace(s, static_cast<scalar_t>(0), _params.h(_params.start, _params.end) * _params.weight);

            // Create buffer:
            stable_forward_buf<node> buf;

            // Main loop:
            while (!open.empty()) {

                auto curr(std::move(open.top()));
                open.pop();

                if (curr.m_index != e) { // SEARCH FOR SOLUTION...

                    closed.allocate(curr.m_index, capacity, _params.maze.count());
                    closed.emplace(curr.m_index);

                    for (const auto& n_data : _params.maze.get_neighbours(curr.m_index)) {

                        if (const auto n = solver_t::get_data(n_data, _params); n.active) {

                            // Check if node is not already visited:
                            if (!closed.contains(n.index)) {
                                 closed.allocate(n.index, capacity, _params.maze.count());
                                 closed.emplace(n.index);
                                   open.emplace(n.index, curr.m_gScore + n.distance, _params.h(n.coord, _params.end) * _params.weight, &buf.emplace(std::move(curr))); // Note: 'current' is now moved!
                            }
                        }
                    }
                }
                else { // SOLUTION REACHED ...

                    result = std::move(curr.template backtrack<node>(_params.size, curr.m_gScore));

                    break;
                }
            }

            return result;
        }

        template <typename open_set_t, size_t Stack>
        [[maybe_unused, nodiscard]] static constexpr auto solve_stack(const params_t& _params) {

            std::vector<coord_t> result;

            const auto s = utils::to_1d(_params.start, _params.size);
            const auto e = utils::to_1d(_params.end,   _params.size);

            // Create closed set:
            const auto capacity = std::max(_params.capacity, std::max(s, e));
            existence_set<low_memory_usage> closed({ s }, capacity);

            // Create open set:
            open_set_t open;
            open.reserve(Stack);
            open.emplace(s, static_cast<scalar_t>(0), _params.h(_params.start, _params.end) * _params.weight);

            // Create buffer:
            stable_forward_buf<node, Stack / 2U> buf;

            // Main loop:
            while (!open.empty()) {

                auto curr(std::move(open.top()));
                open.pop();

                if (curr.m_index != e) { // SEARCH FOR SOLUTION...

                    closed.allocate(curr.m_index, capacity, _params.maze.count());
                    closed.emplace(curr.m_index);

                    for (const auto& n_data : _params.maze.get_neighbours(curr.m_index)) {

                        if (const auto n = solver_t::get_data(n_data, _params); n.active) {

                            // Check if node is not already visited:
                            if (!closed.contains(n.index)) {
                                 closed.allocate(n.index, capacity, _params.maze.count());
                                 closed.emplace(n.index);
                                   open.emplace(n.index, curr.m_gScore + n.distance, _params.h(n.coord, _params.end) * _params.weight, &buf.emplace(std::move(curr))); // Note: 'current' is now moved!
                            }
                        }
                    }
                }
                else { // SOLUTION REACHED ...

                    result = std::move(curr.template backtrack<node>(_params.size, curr.m_gScore));

                    break;
                }
            }

            return result;
        }

        [[maybe_unused, nodiscard]] static constexpr std::vector<coord_t> execute(const params_t& _params) {

            /*
             * Determine whether to solve using a linear search or constant-time
             * method based on which is more efficient given the maze's size.
             */

            constexpr size_t lmax{256U};

            std::vector<coord_t> result;

                 if (_params.maze.count() <=  32U) { constexpr auto stack =       16U; result = solve_stack<linear_priority_queue<node, std::less<node>, std::vector<node, stack_allocator<node, stack>>>, stack>(_params); }
            else if (_params.maze.count() <=  64U) { constexpr auto stack =       32U; result = solve_stack<linear_priority_queue<node, std::less<node>, std::vector<node, stack_allocator<node, stack>>>, stack>(_params); }
            else if (_params.maze.count() <= 128U) { constexpr auto stack =       64U; result = solve_stack<linear_priority_queue<node, std::less<node>, std::vector<node, stack_allocator<node, stack>>>, stack>(_params); }
            else if (_params.maze.count() <= lmax) { constexpr auto stack = lmax / 2U; result = solve_stack<linear_priority_queue<node, std::less<node>, std::vector<node, stack_allocator<node, stack>>>, stack>(_params); }
            else {
                result = solve<heap<node>>(_params);
            }

            return result;
        }
    };

} //chdr::solvers

#endif //CHDR_ASTAR_HPP