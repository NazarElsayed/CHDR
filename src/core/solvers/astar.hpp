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

    template<typename weight_t, const size_t Kd, typename scalar_t, typename index_t>
    class [[maybe_unused]] astar final : public bsolver<weight_t, Kd, scalar_t, index_t> {

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
            [[nodiscard]] constexpr as_node() : unmanaged_node<index_t>() {} // NOLINT(*-pro-type-member-init, *-use-equals-default)

            [[nodiscard]] constexpr as_node(const index_t& _index, const scalar_t& _gScore, const scalar_t& _hScore, const unmanaged_node<index_t>* RESTRICT const _parent) : unmanaged_node<index_t>(_index, _parent),
                m_gScore(_gScore),
                m_fScore(_gScore + _hScore) {}

            struct max {

                [[nodiscard]] constexpr bool operator () (const as_node& _a, const as_node& _b) const {

                    return _a.m_fScore == _b.m_fScore ?
                           _a.m_gScore > _b.m_gScore :
                           _a.m_fScore > _b.m_fScore;
                }
            };
        };

        auto solve_heap(const mazes::graph<index_t, scalar_t>& _maze, const coord_t& _start, const coord_t& _end, const coord_t& _size, scalar_t (*_h)(const coord_t&, const coord_t&), const scalar_t& _weight = 1, size_t _capacity = 0U) const {

            std::vector<coord_t> result;

            const auto s = utils::to_1d(_start, _size);
            const auto e = utils::to_1d(_end,   _size);

            const auto count = _maze.count();

            // Create closed set:
            _capacity = _capacity == 0U ? std::max(_maze.count() / 10U, static_cast<size_t>(1U)) : _capacity;
            existence_set<low_memory_usage> closed({ s }, _capacity);

            // Create open set:
            heap<as_node, typename as_node::max> open(_capacity / 4U);
            open.emplace(s, static_cast<scalar_t>(0), _h(_start, _end), nullptr);

            // Create buffer:
            stable_forward_buf<as_node, 1024U * 1024U> buf;

            // Main loop:
            while (!open.empty()) {

                auto curr(std::move(open.top()));
                open.pop();

                if (curr.m_index != e) { // SEARCH FOR SOLUTION...

                    if (closed.capacity() < curr.m_index) {
                        closed.reserve(std::min(_capacity * ((curr.m_index % _capacity) + 1U), count));
                    }
                    closed.emplace(curr.m_index);

                    for (const auto& neighbour : _maze.get_neighbours(curr.m_index)) {

                        const auto& [n, nDistance] = neighbour;

                        // Check if node is not already visited:
                        if (!closed.contains(n)) {

                            if (closed.capacity() < n) {
                                closed.reserve(std::min(_capacity * ((n % _capacity) + 1U), count));
                            }
                            closed.emplace(n);

                            // Create a parent node and transfer ownership of 'current' to it. Note: 'current' is now moved!
                            open.emplace(n, curr.m_gScore + static_cast<scalar_t>(nDistance), _h(utils::to_nd(n, _size), _end) * _weight, &buf.emplace(std::move(curr)));
                        }
                    }
                }
                else { // SOLUTION REACHED ...

                    curr.template backtrack<as_node>(result, _size, curr.m_gScore);

                    break;
                }
            }

            return result;
        }

        template <size_t StackSize>
        auto solve_linear(const mazes::graph<index_t, scalar_t>& _maze, const coord_t& _start, const coord_t& _end, const coord_t& _size, scalar_t (*_h)(const coord_t&, const coord_t&), const scalar_t& _weight = 1, size_t _capacity = 0U) const {

            std::vector<coord_t> result;

            const auto s = utils::to_1d(_start, _size);
            const auto e = utils::to_1d(_end,   _size);

            const auto count = _maze.count();

            // Create closed set:
            _capacity = std::max(_capacity, std::max(s, e));
            existence_set<low_memory_usage> closed({ s }, _capacity);

            // Create open set:
            linear_priority_queue<as_node, typename as_node::max, std::vector<as_node, stack_allocator<as_node, StackSize>>> open;
            open.reserve(StackSize);
            open.emplace(s, static_cast<scalar_t>(0), _h(_start, _end), nullptr);

            // Create buffer:
            stable_forward_buf<as_node, StackSize / 2U> buf;

            // Main loop:
            while (!open.empty()) {

                auto curr(std::move(open.top()));
                open.pop();

                if (curr.m_index != e) { // SEARCH FOR SOLUTION...

                    if (closed.capacity() < curr.m_index) {
                        closed.reserve(std::min(_capacity * ((curr.m_index % _capacity) + 1U), count));
                    }
                    closed.emplace(curr.m_index);

                    for (const auto& neighbour : _maze.get_neighbours(curr.m_index)) {

                        const auto& [n, nDistance] = neighbour;

                        // Check if node is not already visited:
                        if (!closed.contains(n)) {

                            if (closed.capacity() < n) {
                                closed.reserve(std::min(_capacity * ((n % _capacity) + 1U), count));
                            }
                            closed.emplace(n);

                            // Create a parent node and transfer ownership of 'current' to it. Note: 'current' is now moved!
                            open.emplace(n, curr.m_gScore + static_cast<scalar_t>(nDistance), _h(utils::to_nd(n, _size), _end) * _weight, &buf.emplace(std::move(curr)));
                        }
                    }
                }
                else { // SOLUTION REACHED ...

                    curr.template backtrack<as_node>(result, _size, curr.m_gScore);

                    break;
                }
            }

            return result;
        }

        auto solve_heap(const mazes::grid<Kd, weight_t>& _maze, const coord_t& _start, const coord_t& _end, scalar_t (*_h)(const coord_t&, const coord_t&), const scalar_t& _weight = 1, size_t _capacity = 0U) const {

            std::vector<coord_t> result;

            const auto s = utils::to_1d(_start, _maze.size());
            const auto e = utils::to_1d(_end,   _maze.size());

            const auto count = _maze.count();

            // Create closed set:
            _capacity = std::max(_capacity, std::max(s, e));
            existence_set<low_memory_usage> closed({ s }, _capacity);

            // Create open set:
            heap<as_node, typename as_node::max> open(_capacity / 8U);
            open.emplace(s, static_cast<scalar_t>(0), _h(_start, _end), nullptr);

            // Create buffer:
            stable_forward_buf<as_node> buf;

            // Main loop:
            while (!open.empty()) {

                auto curr(std::move(open.top()));
                open.pop();

                if (curr.m_index != e) { // SEARCH FOR SOLUTION...

                    if (closed.capacity() < curr.m_index) {
                        closed.reserve(std::min(_capacity * ((curr.m_index % _capacity) + 1U), count));
                    }
                    closed.emplace(curr.m_index);

                    for (const auto& neighbour : _maze.get_neighbours(curr.m_index)) {

                        if (const auto& [nActive, nCoord] = neighbour; nActive) {

                            const auto n = utils::to_1d(nCoord, _maze.size());

                            // Check if node is not already visited:
                            if (!closed.contains(n)) {

                                if (closed.capacity() < n) {
                                    closed.reserve(std::min(_capacity * ((n % _capacity) + 1U), count));
                                }
                                closed.emplace(n);

                                // Create a parent node and transfer ownership of 'current' to it. Note: 'current' is now moved!
                                open.emplace(n, curr.m_gScore + static_cast<scalar_t>(1), _h(nCoord, _end) * _weight, &buf.emplace(std::move(curr)));
                            }
                        }
                    }
                }
                else { // SOLUTION REACHED ...

                    curr.template backtrack<as_node>(result, _maze.size(), curr.m_gScore);

                    break;
                }
            }

            return result;
        }

        template <size_t StackSize>
        auto solve_linear(const mazes::grid<Kd, weight_t>& _maze, const coord_t& _start, const coord_t& _end, scalar_t (*_h)(const coord_t&, const coord_t&), const scalar_t& _weight = 1, size_t _capacity = 0U) const {

            std::vector<coord_t> result;

            const auto s = utils::to_1d(_start, _maze.size());
            const auto e = utils::to_1d(_end,   _maze.size());

            const auto count = _maze.count();

            // Create closed set:
            _capacity = std::max(_capacity, std::max(s, e));
            existence_set<low_memory_usage> closed({ s }, _capacity);

            // Create open set:
            linear_priority_queue<as_node, typename as_node::max, std::vector<as_node, stack_allocator<as_node, StackSize>>> open;
            open.reserve(StackSize);
            open.emplace(s, static_cast<scalar_t>(0), _h(_start, _end), nullptr);

            // Create buffer:
            stable_forward_buf<as_node, StackSize / 2U> buf;

            // Main loop:
            while (!open.empty()) {

                auto curr(std::move(open.top()));
                open.pop();

                if (curr.m_index != e) { // SEARCH FOR SOLUTION...

                    if (closed.capacity() < curr.m_index) {
                        closed.reserve(std::min(_capacity * ((curr.m_index % _capacity) + 1U), count));
                    }
                    closed.emplace(curr.m_index);

                    for (const auto& neighbour : _maze.get_neighbours(curr.m_index)) {

                        if (const auto& [nActive, nCoord] = neighbour; nActive) {

                            const auto n = utils::to_1d(nCoord, _maze.size());

                            // Check if node is not already visited:
                            if (!closed.contains(n)) {

                                if (closed.capacity() < n) {
                                    closed.reserve(std::min(_capacity * ((n % _capacity) + 1U), count));
                                }
                                closed.emplace(n);

                                // Create a parent node and transfer ownership of 'current' to it. Note: 'current' is now moved!
                                open.emplace(n, curr.m_gScore + static_cast<scalar_t>(1), _h(nCoord, _end) * _weight, &buf.emplace(std::move(curr)));
                            }
                        }
                    }
                }
                else { // SOLUTION REACHED ...

                    curr.template backtrack<as_node>(result, _maze.size(), curr.m_gScore);

                    break;
                }
            }

            return result;
        }

    public:

        [[maybe_unused]]
        std::vector<coord_t> execute(const mazes::graph<index_t, scalar_t>& _maze, const coord_t& _start, const coord_t& _end, const coord_t& _size, scalar_t (*_h)(const coord_t&, const coord_t&), const scalar_t& _weight, const size_t _capacity) const override {

            /*
             * Determine whether to solve using a linear search or constant-time
             * method based on which is more efficient given the maze's size.
             */

            constexpr size_t lmax = 256U;

            std::vector<coord_t> result;

            const auto count = _maze.count();

            if (count <= 32U) {
                result = solve_linear<16U>(_maze, _start, _end, _size, _h, _weight, _capacity);
            }
            else if (count <= 64U) {
                result = solve_linear<32U>(_maze, _start, _end, _size, _h, _weight, _capacity);
            }
            else if (count <= 128U) {
                result = solve_linear<64U>(_maze, _start, _end, _size, _h, _weight, _capacity);
            }
            else if (count <= lmax) {
                result = solve_linear<lmax / 2U>(_maze, _start, _end, _size, _h, _weight, _capacity);
            }
            else {
                result = solve_heap(_maze, _start, _end, _size, _h, _weight, _capacity);
            }

            return result;
        }

        [[maybe_unused]]
        std::vector<coord_t> execute(const mazes::grid<Kd, weight_t>& _maze, const coord_t& _start, const coord_t& _end, scalar_t (*_h)(const coord_t&, const coord_t&), const scalar_t& _weight, const size_t _capacity) const override {

            /*
             * Determine whether to solve using a linear search or constant-time
             * method based on which is more efficient given the maze's size.
             */

            constexpr size_t lmax = 256U;

            std::vector<coord_t> result;

            const auto count = _maze.count();

            if (count <= 64U) {
                result = solve_linear<32U>(_maze, _start, _end, _h, _weight, _capacity);
            }
            else if (count <= 128U) {
                result = solve_linear<64U>(_maze, _start, _end, _h, _weight, _capacity);
            }
            else if (count <= lmax) {
                result = solve_linear<lmax / 2U>(_maze, _start, _end, _h, _weight, _capacity);
            }
            else {
                result = solve_heap(_maze, _start, _end, _h, _weight, _capacity);
            }

            return result;
        }
    };

} //chdr::solvers

#endif //CHDR_ASTAR_HPP