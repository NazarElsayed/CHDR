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
#include "base/bsolver.hpp"
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

    template<typename weight_t, const size_t Kd, typename scalar_t, typename index_t, typename params_t>
    class [[maybe_unused]] astar final : public bsolver<weight_t, Kd, scalar_t, index_t, params_t> {

        static_assert(std::is_integral_v<scalar_t> || std::is_floating_point_v<scalar_t>, "scalar_t must be either an integral or floating point type.");
        static_assert(std::is_integral_v<index_t>, "index_t must be an integral type.");

    private:

        using coord_t = coord<index_t, Kd>;

        struct as_node final : unmanaged_node<index_t> {

            scalar_t m_gScore;
            scalar_t m_fScore;

            /**
             * @brief Constructs an uninitialized ASNode.
             *
             * This constructor creates an ASNode with uninitialized members.
             */
            [[nodiscard]] constexpr as_node() noexcept : unmanaged_node<index_t>() {} // NOLINT(*-pro-type-member-init, *-use-equals-default)

            [[nodiscard]] constexpr as_node(const index_t& _index, const scalar_t& _gScore, const scalar_t& _hScore, const unmanaged_node<index_t>* RESTRICT const _parent) noexcept : unmanaged_node<index_t>(_index, _parent),
                m_gScore(_gScore),
                m_fScore(_gScore + _hScore) {}

            struct max {

                [[nodiscard]] constexpr bool operator () (const as_node& _a, const as_node& _b) const noexcept {

                    return _a.m_fScore == _b.m_fScore ?
                           _a.m_gScore >  _b.m_gScore :
                           _a.m_fScore >  _b.m_fScore;
                }
            };
        };

        [[maybe_unused, nodiscard]] auto solve_heap(const params_t& _params) const {

            std::vector<coord_t> result;

            const auto s = utils::to_1d(_params._start, _params._maze.size());
            const auto e = utils::to_1d(_params._end,   _params._maze.size());

            // Create closed set:
            const auto capacity = std::max(_params._capacity, std::max(s, e));
            existence_set<low_memory_usage> closed({ s }, capacity);

            // Create open set:
            heap<as_node, typename as_node::max> open(capacity / 8U);
            open.emplace(s, static_cast<scalar_t>(0), _params._h(_params._start, _params._end), nullptr);

            // Create buffer:
            stable_forward_buf<as_node> buf;

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
                                open.emplace(n, curr.m_gScore + static_cast<scalar_t>(nDistance), _params._h(nCoord, _params._end) * _params._weight, &buf.emplace(std::move(curr)));
                            }
                        }
                        else {

                            if (const auto& [nActive, nCoord] = neighbour; nActive) {

                                const auto n = utils::to_1d(nCoord, _params._maze.size());

                                constexpr auto nDistance = static_cast<scalar_t>(1);

                                // Check if node is not already visited:
                                if (!closed.contains(n)) {
                                     closed.allocate(n, capacity, _params._maze.count());
                                     closed.emplace(n);

                                    // Create a parent node and transfer ownership of 'current' to it. Note: 'current' is now moved!
                                    open.emplace(n, curr.m_gScore + static_cast<scalar_t>(nDistance), _params._h(nCoord, _params._end) * _params._weight, &buf.emplace(std::move(curr)));
                                }
                            }
                        }
                    }
                }
                else { // SOLUTION REACHED ...

                    result = std::move(curr.template backtrack<as_node>(_params._maze.size(), curr.m_gScore));

                    break;
                }
            }

            return result;
        }

        template <size_t StackSize>
        [[maybe_unused, nodiscard]] auto solve_linear(const params_t& _params) const {

            std::vector<coord_t> result;

            const auto s = utils::to_1d(_params._start, _params._maze.size());
            const auto e = utils::to_1d(_params._end,   _params._maze.size());

            // Create closed set:
            const auto capacity = std::max(_params._capacity, std::max(s, e));
            existence_set<low_memory_usage> closed({ s }, capacity);

            // Create open set:
            linear_priority_queue<as_node, typename as_node::max, std::vector<as_node, stack_allocator<as_node, StackSize>>> open;
            open.reserve(StackSize);
            open.emplace(s, static_cast<scalar_t>(0), _params._h(_params._start, _params._end), nullptr);

            // Create buffer:
            stable_forward_buf<as_node, StackSize / 2U> buf;

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
                                open.emplace(n, curr.m_gScore + static_cast<scalar_t>(nDistance), _params._h(nCoord, _params._end) * _params._weight, &buf.emplace(std::move(curr)));
                            }
                        }
                        else {

                            if (const auto& [nActive, nCoord] = neighbour; nActive) {

                                const auto n = utils::to_1d(nCoord, _params._maze.size());

                                const auto nDistance = static_cast<scalar_t>(1);

                                // Check if node is not already visited:
                                if (!closed.contains(n)) {
                                     closed.allocate(n, capacity, _params._maze.count());
                                     closed.emplace(n);

                                    // Create a parent node and transfer ownership of 'current' to it. Note: 'current' is now moved!
                                    open.emplace(n, curr.m_gScore + static_cast<scalar_t>(nDistance), _params._h(nCoord, _params._end) * _params._weight, &buf.emplace(std::move(curr)));
                                }
                            }
                        }
                    }
                }
                else { // SOLUTION REACHED ...

                    result = std::move(curr.template backtrack<as_node>(_params._maze.size(), curr.m_gScore));

                    break;
                }
            }

            return result;
        }

    public:

        [[maybe_unused, nodiscard]] std::vector<coord_t> execute(const params_t& _params) const override {

            /*
             * Determine whether to solve using a linear search or constant-time
             * method based on which is more efficient given the maze's size.
             */

            constexpr size_t lmax = 256U;

            std::vector<coord_t> result;

                 if (_params._maze.count() <=  32U) { result = solve_linear<16U>(_params); }
            else if (_params._maze.count() <=  64U) { result = solve_linear<32U>(_params); }
            else if (_params._maze.count() <= 128U) { result = solve_linear<64U>(_params); }
            else if (_params._maze.count() <= lmax) { result = solve_linear<lmax / 2U>(_params); }
            else {
                result = solve_heap(_params);
            }

            return result;
        }
    };

} //chdr::solvers

#endif //CHDR_ASTAR_HPP