/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_BSTAR_HPP
#define CHDR_BSTAR_HPP

#include "base/bsolver.hpp"
#include "mazes/graph.hpp"
#include "mazes/Grid.hpp"
#include "solvers/base/UnmanagedNode.hpp"
#include "types/existence_set.hpp"
#include "types/heap.hpp"
#include "types/StableForwardBuf.hpp"
#include "types/StackAllocator.hpp"
#include "utils/Utils.hpp"

namespace chdr::solvers {

    template<typename weight_t, const size_t Kd, typename scalar_t, typename index_t>
    class [[maybe_unused]] bstar final : public bsolver<weight_t, Kd, scalar_t, index_t> {

        static_assert(std::is_integral_v<scalar_t> || std::is_floating_point_v<scalar_t>, "scalar_t must be either an integral or floating point type.");
        static_assert(std::is_integral_v<index_t>, "index_t must be an integral type.");

    private:

        using coord_t = coord_t<index_t, Kd>;

        struct bs_node final : public UnmanagedNode<index_t> {

            scalar_t m_hScore;

            /**
             * @brief Constructs an uninitialized BSNode.
             *
             * This constructor creates an BSNode with uninitialized members.
             */
            [[nodiscard]] constexpr bs_node() {} // NOLINT(*-pro-type-member-init, *-use-equals-default)

            [[nodiscard]] constexpr bs_node(const index_t& _index, const scalar_t& _hScore, const UnmanagedNode<index_t>* RESTRICT const _parent) : UnmanagedNode<index_t>(_index, _parent),
                                                                                                                                                    m_hScore(_hScore) {}

            struct max {

                [[nodiscard]] constexpr bool operator () (const bs_node& _a, const bs_node& _b) const {
                    return _a.m_hScore > _b.m_hScore;
                }
            };

            struct min {

                [[nodiscard]] constexpr bool operator () (const bs_node& _a, const bs_node& _b) const {
                    return _a.m_hScore < _b.m_hScore;
                }
            };
        };

        auto solveHeap(const mazes::graph<index_t, scalar_t>& _maze, const coord_t& _start, const coord_t& _end, const coord_t& _size, scalar_t (*_h)(const coord_t&, const coord_t&), size_t _capacity = 0U) const {

            std::vector<coord_t> result;

            const auto s = Utils::To1D(_start, _size);
            const auto e = Utils::To1D(_end,   _size);

            const auto count = _maze.Count();

            // Create closed set:
            _capacity = _capacity == 0U ? std::max(_maze.Count() / 10U, static_cast<size_t>(1U)) : _capacity;
            existence_set<low_memory_usage> closed({s }, _capacity);

            // Create open set:
            heap<bs_node, 2U, typename bs_node::max> open(_capacity / 4U);
            open.Emplace({ s, _h(_start, _end), nullptr });

            // Create buffer:
            StableForwardBuf<bs_node, 1024U * 1024U> buf;

            // Main loop:
            while (!open.Empty()) {

                auto curr = open.PopTop();

                if (curr.m_index != e) { // SEARCH FOR SOLUTION...

                    if (closed.capacity() < curr.m_index) {
                        closed.reserve(std::min(_capacity * ((curr.m_index % _capacity) + 1U), count));
                    }
                    closed.add(curr.m_index);

                    for (const auto& neighbour : _maze.GetNeighbours(curr.m_index)) {

                        const auto& [n, nDistance] = neighbour;

                        // Check if node is not already visited:
                        if (!closed.contains(n)) {

                            if (closed.capacity() < n) {
                                closed.reserve(std::min(_capacity * ((n % _capacity) + 1U), count));
                            }
                            closed.add(n);

                            // Create a parent node and transfer ownership of 'current' to it. Note: 'current' is now moved!
                            open.Emplace({n, _h(chdr::Utils::ToND(n, _size), _end), &buf.Emplace(std::move(curr)) });
                        }
                    }
                }
                else { // SOLUTION REACHED ...

                    curr.template Backtrack<bs_node>(result, _size, static_cast<size_t>(_h(_start, _end)));

                    break;
                }
            }

            return result;
        }

        template <size_t StackSize>
        auto solveLinear(const mazes::graph<index_t, scalar_t>& _maze, const coord_t& _start, const coord_t& _end, const coord_t& _size, scalar_t (*_h)(const coord_t&, const coord_t&), size_t _capacity = 0U) const {

            std::vector<coord_t> result;

            const auto s = Utils::To1D(_start, _size);
            const auto e = Utils::To1D(_end,   _size);

            const auto count = _maze.Count();

            // Create closed set:
            _capacity = std::max(_capacity, std::max(s, e));
            existence_set<low_memory_usage> closed({s }, _capacity);

            // Create open set:
            std::vector<bs_node, StackAllocator<bs_node, StackSize>> open;
            open.reserve(StackSize);
            open.push_back({ s, _h(_start, _end), nullptr });

            // Create buffer:
            StableForwardBuf<bs_node, StackSize / 2U> buf;

            // Main loop:
            while (!open.empty()) {

                const auto top = std::min_element(open.begin(), open.end(), typename bs_node::min());
                auto curr(std::move(*top));
                open.erase(top);

                if (curr.m_index != e) { // SEARCH FOR SOLUTION...

                    if (closed.capacity() < curr.m_index) {
                        closed.reserve(std::min(_capacity * ((curr.m_index % _capacity) + 1U), count));
                    }
                    closed.add(curr.m_index);

                    for (const auto& neighbour : _maze.GetNeighbours(curr.m_index)) {

                        const auto& [n, nDistance] = neighbour;

                        // Check if node is not already visited:
                        if (!closed.contains(n)) {

                            if (closed.capacity() < n) {
                                closed.reserve(std::min(_capacity * ((n % _capacity) + 1U), count));
                            }
                            closed.add(n);

                            // Create a parent node and transfer ownership of 'current' to it. Note: 'current' is now moved!
                            open.push_back({n, _h(chdr::Utils::ToND(n, _size), _end), &buf.Emplace(std::move(curr)) });
                        }
                    }
                }
                else { // SOLUTION REACHED ...

                    curr.template Backtrack<bs_node>(result, _size, static_cast<size_t>(_h(_start, _end)));

                    break;
                }
            }

            return result;
        }

        auto solveHeap(const mazes::Grid<Kd, weight_t>& _maze, const coord_t& _start, const coord_t& _end, scalar_t (*_h)(const coord_t&, const coord_t&), size_t _capacity = 0U) const {

            std::vector<coord_t> result;

            const auto s = Utils::To1D(_start, _maze.Size());
            const auto e = Utils::To1D(_end,   _maze.Size());

            const auto count = _maze.Count();

            // Create closed set:
            _capacity = std::max(_capacity, std::max(s, e));
            existence_set<low_memory_usage> closed({s }, _capacity);

            // Create open set:
            heap<bs_node, 2U, typename bs_node::max> open(_capacity / 8U);
            open.Emplace({ s, _h(_start, _end), nullptr });

            // Create buffer:
            StableForwardBuf<bs_node> buf;

            // Main loop:
            while (!open.Empty()) {

                auto curr = open.PopTop();

                if (curr.m_index != e) { // SEARCH FOR SOLUTION...

                    if (closed.capacity() < curr.m_index) {
                        closed.reserve(std::min(_capacity * ((curr.m_index % _capacity) + 1U), count));
                    }
                    closed.add(curr.m_index);

                    for (const auto& neighbour : _maze.GetNeighbours(curr.m_index)) {

                        if (const auto& [nActive, nCoord] = neighbour; nActive) {

                            const auto n = Utils::To1D(nCoord, _maze.Size());

                            // Check if node is not already visited:
                            if (!closed.contains(n)) {

                                if (closed.capacity() < n) {
                                    closed.reserve(std::min(_capacity * ((n % _capacity) + 1U), count));
                                }
                                closed.add(n);

                                // Create a parent node and transfer ownership of 'current' to it. Note: 'current' is now moved!
                                open.Emplace({ n, _h(nCoord, _end), &buf.Emplace(std::move(curr)) });
                            }
                        }
                    }
                }
                else { // SOLUTION REACHED ...

                    curr.template Backtrack<bs_node>(result, _maze.Size(), static_cast<size_t>(_h(_start, _end)));

                    break;
                }
            }

            return result;
        }

        template <size_t StackSize>
        auto solveLinear(const mazes::Grid<Kd, weight_t>& _maze, const coord_t& _start, const coord_t& _end, scalar_t (*_h)(const coord_t&, const coord_t&), size_t _capacity = 0U) const {

            std::vector<coord_t> result;

            const auto s = Utils::To1D(_start, _maze.Size());
            const auto e = Utils::To1D(_end,   _maze.Size());

            const auto count = _maze.Count();

            // Create closed set:
            _capacity = std::max(_capacity, std::max(s, e));
            existence_set<low_memory_usage> closed({s }, _capacity);

            // Create open set:
            std::vector<bs_node, StackAllocator<bs_node, StackSize>> open;
            open.reserve(StackSize);
            open.push_back({ s, _h(_start, _end), nullptr });

            // Create buffer:
            StableForwardBuf<bs_node, StackSize / 2U> buf;

            // Main loop:
            while (!open.empty()) {

                const auto top = std::min_element(open.begin(), open.end(), typename bs_node::min());
                auto curr(std::move(*top));
                open.erase(top);

                if (curr.m_index != e) { // SEARCH FOR SOLUTION...

                    if (closed.capacity() < curr.m_index) {
                        closed.reserve(std::min(_capacity * ((curr.m_index % _capacity) + 1U), count));
                    }
                    closed.add(curr.m_index);

                    for (const auto& neighbour : _maze.GetNeighbours(curr.m_index)) {

                        if (const auto& [nActive, nCoord] = neighbour; nActive) {

                            const auto n = Utils::To1D(nCoord, _maze.Size());

                            // Check if node is not already visited:
                            if (!closed.contains(n)) {

                                if (closed.capacity() < n) {
                                    closed.reserve(std::min(_capacity * ((n % _capacity) + 1U), count));
                                }
                                closed.add(n);

                                // Create a parent node and transfer ownership of 'current' to it. Note: 'current' is now moved!
                                open.push_back({ n, _h(nCoord, _end), &buf.Emplace(std::move(curr)) });
                            }
                        }
                    }
                }
                else { // SOLUTION REACHED ...

                    curr.template Backtrack<bs_node>(result, _maze.Size(), static_cast<size_t>(_h(_start, _end)));

                    break;
                }
            }

            return result;
        }

    public:

        [[maybe_unused]]
        std::vector<coord_t> execute(const mazes::graph<index_t, scalar_t>& _maze, const coord_t& _start, const coord_t& _end, const coord_t& _size, scalar_t (*_h)(const coord_t&, const coord_t&), const scalar_t& _weight, size_t _capacity) const override {

            /*
             * Determine whether to solve using a linear search or constant-time
             * method based on which is more efficient given the maze's size.
             */

            constexpr size_t lmax = 256U;

            std::vector<coord_t> result;

            const auto count = _maze.Count();

            if (count <= 32U) {
                result = SolveLinear<16U>(_maze, _start, _end, _size, _h, _capacity);
            }
            else if (count <= 64U) {
                result = SolveLinear<32U>(_maze, _start, _end, _size, _h, _capacity);
            }
            else if (count <= 128U) {
                result = SolveLinear<64U>(_maze, _start, _end, _size, _h, _capacity);
            }
            else if (count <= lmax) {
                result = SolveLinear<lmax / 2U>(_maze, _start, _end, _size, _h, _capacity);
            }
            else {
                result = solveHeap(_maze, _start, _end, _size, _h, _capacity);
            }

            return result;
        }

        [[maybe_unused]]
        std::vector<coord_t> execute(const mazes::Grid<Kd, weight_t>& _maze, const coord_t& _start, const coord_t& _end, scalar_t (*_h)(const coord_t&, const coord_t&), const scalar_t& _weight, size_t _capacity) const override {

            /*
             * Determine whether to solve using a linear search or constant-time
             * method based on which is more efficient given the maze's size.
             */

            constexpr size_t lmax = 256U;

            std::vector<coord_t> result;

            const auto count = _maze.Count();

            if (count <= 64U) {
                result = SolveLinear<32U>(_maze, _start, _end, _h, _capacity);
            }
            else if (count <= 128U) {
                result = SolveLinear<64U>(_maze, _start, _end, _h, _capacity);
            }
            else if (count <= lmax) {
                result = SolveLinear<lmax / 2U>(_maze, _start, _end, _h, _capacity);
            }
            else {
                result = solveHeap(_maze, _start, _end, _h, _capacity);
            }

            return result;
        }
    };

} // CHDR::Solvers

#endif //CHDR_BSTAR_HPP