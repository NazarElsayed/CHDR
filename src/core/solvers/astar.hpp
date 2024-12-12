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

        using coord_t = coord<index_t, Kd>;

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

        template <typename heap_t>
        [[maybe_unused, nodiscard]] static constexpr auto solve(const params_t& _params) {

            std::vector<coord_t> result;

            const auto s = utils::to_1d(_params._start, _params._size);
            const auto e = utils::to_1d(_params._end,   _params._size);

            // Create closed set:
            const auto capacity = std::max(_params._capacity, std::max(s, e));
            existence_set<low_memory_usage> closed({ s }, capacity);

            // Create open set:
            heap_t open(capacity / 8U);
            open.emplace(s, static_cast<scalar_t>(0), _params._h(_params._start, _params._end));

            // Create buffer:
            stable_forward_buf<node> buf;

            // Main loop:
            while (!open.empty()) {

                auto curr(std::move(open.top()));
                open.pop();

                if (curr.m_index != e) { // SEARCH FOR SOLUTION...

                    closed.allocate(curr.m_index, capacity, _params._maze.count());
                    closed.emplace(curr.m_index);

                    for (const auto& neighbour : _params._maze.get_neighbours(curr.m_index)) {

                        if constexpr (std::is_same_v<std::decay_t<decltype(_params._maze)>, mazes::graph<index_t, scalar_t>>) {

                            const auto& [n, nDistance] = neighbour;

                            const auto nCoord = utils::to_nd(n, _params._size);

                            // Check if node is not already visited:
                            if (!closed.contains(n)) {
                                 closed.allocate(n, capacity, _params._maze.count());
                                 closed.emplace(n);

                                // Create a parent node and transfer ownership of 'current' to it. Note: 'current' is now moved!
                                open.emplace(n, curr.m_gScore + nDistance, _params._h(nCoord, _params._end) * _params._weight, &buf.emplace(std::move(curr)));
                            }
                        }
                        else {

                            if (const auto& [nActive, nCoord] = neighbour; nActive) {

                                const auto n = utils::to_1d(nCoord, _params._size);

                                constexpr scalar_t nDistance{1};

                                // Check if node is not already visited:
                                if (!closed.contains(n)) {
                                     closed.allocate(n, capacity, _params._maze.count());
                                     closed.emplace(n);

                                    // Create a parent node and transfer ownership of 'current' to it. Note: 'current' is now moved!
                                    open.emplace(n, curr.m_gScore + nDistance, _params._h(nCoord, _params._end) * _params._weight, &buf.emplace(std::move(curr)));
                                }
                            }
                        }
                    }
                }
                else { // SOLUTION REACHED ...

                    result = std::move(curr.template backtrack<node>(_params._size, curr.m_gScore));

                    break;
                }
            }

            return result;
        }

        template <typename heap_t, size_t Stack>
        [[maybe_unused, nodiscard]] static constexpr auto solve_stack(const params_t& _params) {

            std::vector<coord_t> result;

            const auto s = utils::to_1d(_params._start, _params._size);
            const auto e = utils::to_1d(_params._end,   _params._size);

            // Create closed set:
            const auto capacity = std::max(_params._capacity, std::max(s, e));
            existence_set<low_memory_usage> closed({ s }, capacity);

            // Create open set:
            heap_t open;
            open.reserve(Stack);
            open.emplace(s, static_cast<scalar_t>(0), _params._h(_params._start, _params._end));

            // Create buffer:
            stable_forward_buf<node, Stack / 2U> buf;

            // Main loop:
            while (!open.empty()) {

                auto curr(std::move(open.top()));
                open.pop();

                if (curr.m_index != e) { // SEARCH FOR SOLUTION...

                    closed.allocate(curr.m_index, capacity, _params._maze.count());
                    closed.emplace(curr.m_index);

                    for (const auto& neighbour : _params._maze.get_neighbours(curr.m_index)) {

                        if constexpr (std::is_same_v<std::decay_t<decltype(_params._maze)>, mazes::graph<index_t, scalar_t>>) {

                            const auto& [n, nDistance] = neighbour;

                            const auto nCoord = utils::to_nd(n, _params._size);

                            // Check if node is not already visited:
                            if (!closed.contains(n)) {
                                 closed.allocate(n, capacity, _params._maze.count());
                                 closed.emplace(n);
                                   open.emplace(n, curr.m_gScore + nDistance, _params._h(nCoord, _params._end) * _params._weight, &buf.emplace(std::move(curr))); // Note: 'current' is now moved!
                            }
                        }
                        else {

                            if (const auto& [nActive, nCoord] = neighbour; nActive) {

                                const auto n = utils::to_1d(nCoord, _params._size);

                                constexpr scalar_t nDistance{1};

                                // Check if node is not already visited:
                                if (!closed.contains(n)) {
                                     closed.allocate(n, capacity, _params._maze.count());
                                     closed.emplace(n);
                                       open.emplace(n, curr.m_gScore + nDistance, _params._h(nCoord, _params._end) * _params._weight, &buf.emplace(std::move(curr))); // Note: 'current' is now moved!
                                }
                            }
                        }
                    }
                }
                else { // SOLUTION REACHED ...

                    result = std::move(curr.template backtrack<node>(_params._size, curr.m_gScore));

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

                 if (_params._maze.count() <=  32U) { constexpr auto stack =       16U; result = solve_stack<linear_priority_queue<node, std::less<node>, std::vector<node, stack_allocator<node, stack>>>, stack>(_params); }
            else if (_params._maze.count() <=  64U) { constexpr auto stack =       32U; result = solve_stack<linear_priority_queue<node, std::less<node>, std::vector<node, stack_allocator<node, stack>>>, stack>(_params); }
            else if (_params._maze.count() <= 128U) { constexpr auto stack =       64U; result = solve_stack<linear_priority_queue<node, std::less<node>, std::vector<node, stack_allocator<node, stack>>>, stack>(_params); }
            else if (_params._maze.count() <= lmax) { constexpr auto stack = lmax / 2U; result = solve_stack<linear_priority_queue<node, std::less<node>, std::vector<node, stack_allocator<node, stack>>>, stack>(_params); }
            else {
                result = solve<heap<node>>(_params);
            }

            return result;
        }
    };

} //chdr::solvers

#endif //CHDR_ASTAR_HPP