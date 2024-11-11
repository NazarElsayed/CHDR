/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_ASTAR_HPP
#define CHDR_ASTAR_HPP

#include "../utils/Heuristics.hpp"
#include "base/BSolver.hpp"
#include "mazes/base/IMaze.hpp"
#include "mazes/Graph.hpp"
#include "mazes/Grid.hpp"
#include "solvers/base/UnmanagedNode.hpp"
#include "types/ExistenceSet.hpp"
#include "types/Heap.hpp"
#include "types/StableForwardBuf.hpp"
#include "types/StackAllocator.hpp"
#include "utils/Utils.hpp"

namespace CHDR::Solvers {

    template<typename weight_t, const size_t Kd, typename scalar_t, typename index_t>
    class [[maybe_unused]] AStar final : public BSolver<weight_t, Kd, scalar_t, index_t> {

        static_assert(std::is_integral_v<scalar_t> || std::is_floating_point_v<scalar_t>, "scalar_t must be either an integral or floating point type.");
        static_assert(std::is_integral_v<index_t>, "index_t must be an integral type.");

    private:

        using coord_t = Coord<index_t, Kd>;

        struct ASNode final : public UnmanagedNode<index_t> {

            scalar_t m_GScore;
            scalar_t m_FScore;

            /**
             * @brief Constructs an uninitialized ASNode.
             *
             * This constructor creates an ASNode with uninitialized members.
             */
            [[nodiscard]] constexpr ASNode() : UnmanagedNode<index_t>() {} // NOLINT(*-pro-type-member-init, *-use-equals-default)

            [[nodiscard]] constexpr ASNode(const index_t& _index, const scalar_t& _gScore, const scalar_t& _hScore, const UnmanagedNode<index_t>* RESTRICT const _parent) : UnmanagedNode<index_t>(_index, _parent),
                m_GScore(_gScore),
                m_FScore(_gScore + _hScore) {}

            struct Max {

                [[nodiscard]] constexpr bool operator () (const ASNode& _a, const ASNode& _b) const {

                    return _a.m_FScore == _b.m_FScore ?
                        _a.m_GScore > _b.m_GScore :
                        _a.m_FScore > _b.m_FScore;
                }
            };

            struct Min {

                [[nodiscard]] constexpr bool operator () (const ASNode& _a, const ASNode& _b) const {

                    return _a.m_FScore == _b.m_FScore ?
                        _a.m_GScore < _b.m_GScore :
                        _a.m_FScore < _b.m_FScore;
                }
            };
        };

        auto SolveHeap(const Mazes::Graph<index_t, scalar_t>& _maze, const coord_t& _start, const coord_t& _end, const coord_t& _size, scalar_t (*_h)(const coord_t&, const coord_t&), const scalar_t& _weight = 1, size_t _capacity = 0U) const {

            std::vector<coord_t> result;

            const auto s = Utils::To1D(_start, _size);
            const auto e = Utils::To1D(_end,   _size);

            const auto count = _maze.Count();

            // Create closed set:
            _capacity = _capacity == 0U ? std::max(_maze.Count() / 10U, static_cast<size_t>(1U)) : _capacity;
            ExistenceSet<LowMemoryUsage> closed({ s }, _capacity);

            // Create open set:
            Heap<ASNode, 2U, typename ASNode::Max> open(_capacity / 4U);
            open.Emplace({s, static_cast<scalar_t>(0), _h(_start, _end), nullptr });

            // Create buffer:
            StableForwardBuf<ASNode, 1024U * 1024U> buf;

            // Main loop:
            while (!open.Empty()) {

                auto curr = open.PopTop();

                if (curr.m_Index != e) { // SEARCH FOR SOLUTION...

                    if (closed.Capacity() < curr.m_Index) {
                        closed.Reserve(std::min(_capacity * ((curr.m_Index % _capacity) + 1U), count));
                    }
                    closed.Add(curr.m_Index);

                    for (const auto& neighbour : _maze.GetNeighbours(curr.m_Index)) {

                        const auto& [n, nDistance] = neighbour;

                        // Check if node is not already visited:
                        if (!closed.Contains(n)) {

                            if (closed.Capacity() < n) {
                                closed.Reserve(std::min(_capacity * ((n % _capacity) + 1U), count));
                            }
                            closed.Add(n);

                            // Create a parent node and transfer ownership of 'current' to it. Note: 'current' is now moved!
                            open.Emplace({ n, curr.m_GScore + static_cast<scalar_t>(nDistance), _h(CHDR::Utils::ToND(n, _size), _end) * _weight, &buf.Emplace(std::move(curr)) });
                        }
                    }
                }
                else { // SOLUTION REACHED ...

                    // Reserve space in result:
                    result.reserve(curr.m_GScore);

                    // Recurse from end node to start node, inserting into a result buffer:
                    for (const auto* temp = &curr; temp->m_Parent != nullptr; temp = static_cast<const ASNode*>(temp->m_Parent)) {
                        result.emplace_back(Utils::ToND(temp->m_Index, _size));
                    }

                    // Reverse the result:
                    std::reverse(result.begin(), result.end());

                    break;
                }
            }

            return result;
        }

        template <size_t StackSize>
        auto SolveLinear(const Mazes::Graph<index_t, scalar_t>& _maze, const coord_t& _start, const coord_t& _end, const coord_t& _size, scalar_t (*_h)(const coord_t&, const coord_t&), const scalar_t& _weight = 1, size_t _capacity = 0U) const {

            std::vector<coord_t> result;

            const auto s = Utils::To1D(_start, _size);
            const auto e = Utils::To1D(_end,   _size);

            const auto count = _maze.Count();

            // Create closed set:
            _capacity = std::max(_capacity, std::max(s, e));
            ExistenceSet<LowMemoryUsage> closed({ s }, _capacity);

            // Create open set:
            std::vector<ASNode, StackAllocator<ASNode, StackSize>> open;
            open.reserve(StackSize);
            open.push_back({ s, static_cast<scalar_t>(0), _h(_start, _end), nullptr });

            // Create buffer:
            StableForwardBuf<ASNode, StackSize / 2U> buf;

            // Main loop:
            while (!open.empty()) {

                const auto top = std::min_element(open.begin(), open.end(), typename ASNode::Min());
                auto curr(std::move(*top));
                open.erase(top);

                if (curr.m_Index != e) { // SEARCH FOR SOLUTION...

                    if (closed.Capacity() < curr.m_Index) {
                        closed.Reserve(std::min(_capacity * ((curr.m_Index % _capacity) + 1U), count));
                    }
                    closed.Add(curr.m_Index);

                    for (const auto& neighbour : _maze.GetNeighbours(curr.m_Index)) {

                        const auto& [n, nDistance] = neighbour;

                        // Check if node is not already visited:
                        if (!closed.Contains(n)) {

                            if (closed.Capacity() < n) {
                                closed.Reserve(std::min(_capacity * ((n % _capacity) + 1U), count));
                            }
                            closed.Add(n);

                            // Create a parent node and transfer ownership of 'current' to it. Note: 'current' is now moved!
                            open.push_back({ n, curr.m_GScore + static_cast<scalar_t>(nDistance), _h(CHDR::Utils::ToND(n, _size), _end) * _weight, &buf.Emplace(std::move(curr)) });
                        }
                    }
                }
                else { // SOLUTION REACHED ...

                    // Reserve space in result:
                    result.reserve(curr.m_GScore);

                    // Recurse from end node to start node, inserting into a result buffer:
                    for (const auto* temp = &curr; temp->m_Parent != nullptr; temp = static_cast<const ASNode*>(temp->m_Parent)) {
                        result.emplace_back(Utils::ToND(temp->m_Index, _size));
                    }

                    // Reverse the result:
                    std::reverse(result.begin(), result.end());

                    break;
                }
            }

            return result;
        }

        auto SolveHeap(const Mazes::Grid<Kd, weight_t>& _maze, const coord_t& _start, const coord_t& _end, scalar_t (*_h)(const coord_t&, const coord_t&), const scalar_t& _weight = 1, size_t _capacity = 0U) const {

            std::vector<coord_t> result;

            const auto s = Utils::To1D(_start, _maze.Size());
            const auto e = Utils::To1D(_end,   _maze.Size());

            const auto count = _maze.Count();

            // Create closed set:
            _capacity = std::max(_capacity, std::max(s, e));
            ExistenceSet<LowMemoryUsage> closed({ s }, _capacity);

            // Create open set:
            Heap<ASNode, 2U, typename ASNode::Max> open(_capacity / 8U);
            open.Emplace({  s, static_cast<scalar_t>(0), _h(_start, _end), nullptr });

            // Create buffer:
            StableForwardBuf<ASNode> buf;

            // Main loop:
            while (!open.Empty()) {
                auto curr = open.PopTop();

                if (curr.m_Index != e) { // SEARCH FOR SOLUTION...

                    if (closed.Capacity() < curr.m_Index) {
                        closed.Reserve(std::min(_capacity * ((curr.m_Index % _capacity) + 1U), count));
                    }
                    closed.Add(curr.m_Index);

                    for (const auto& neighbour : _maze.GetNeighbours(curr.m_Index)) {

                        if (const auto& [nActive, nCoord] = neighbour; nActive) {

                            const auto n = Utils::To1D(nCoord, _maze.Size());

                            // Check if node is not already visited:
                            if (!closed.Contains(n)) {

                                if (closed.Capacity() < n) {
                                    closed.Reserve(std::min(_capacity * ((n % _capacity) + 1U), count));
                                }
                                closed.Add(n);

                                // Create a parent node and transfer ownership of 'current' to it. Note: 'current' is now moved!
                                open.Emplace({ n, curr.m_GScore + static_cast<scalar_t>(1), _h(nCoord, _end) * _weight, &buf.Emplace(std::move(curr)) });
                            }
                        }
                    }
                }
                else { // SOLUTION REACHED ...

                    // Reserve space in result:
                    result.reserve(curr.m_GScore);

                    // Recurse from end node to start node, inserting into a result buffer:
                    for (const auto* temp = &curr; temp->m_Parent != nullptr; temp = static_cast<const ASNode*>(temp->m_Parent)) {
                        result.emplace_back(Utils::ToND(temp->m_Index, _maze.Size()));
                    }

                    // Reverse the result:
                    std::reverse(result.begin(), result.end());

                    break;
                }
            }

            return result;
        }

        template <size_t StackSize>
        auto SolveLinear(const Mazes::Grid<Kd, weight_t>& _maze, const coord_t& _start, const coord_t& _end, scalar_t (*_h)(const coord_t&, const coord_t&), const scalar_t& _weight = 1, size_t _capacity = 0U) const {

            std::vector<coord_t> result;

            const auto s = Utils::To1D(_start, _maze.Size());
            const auto e = Utils::To1D(_end,   _maze.Size());

            const auto count = _maze.Count();

            // Create closed set:
            _capacity = std::max(_capacity, std::max(s, e));
            ExistenceSet<LowMemoryUsage> closed({ s }, _capacity);

            // Create open set:
            std::vector<ASNode, StackAllocator<ASNode, StackSize>> open;
            open.reserve(StackSize);
            open.push_back({ s, static_cast<scalar_t>(0), _h(_start, _end), nullptr });

            // Create buffer:
            StableForwardBuf<ASNode, StackSize / 2U> buf;

            // Main loop:
            while (!open.empty()) {
                const auto top = std::min_element(open.begin(), open.end(), typename ASNode::Min());
                auto curr(std::move(*top));
                open.erase(top);

                if (curr.m_Index != e) { // SEARCH FOR SOLUTION...

                    if (closed.Capacity() < curr.m_Index) {
                        closed.Reserve(std::min(_capacity * ((curr.m_Index % _capacity) + 1U), count));
                    }
                    closed.Add(curr.m_Index);

                    for (const auto& neighbour : _maze.GetNeighbours(curr.m_Index)) {

                        if (const auto& [nActive, nCoord] = neighbour; nActive) {

                            const auto n = Utils::To1D(nCoord, _maze.Size());

                            // Check if node is not already visited:
                            if (!closed.Contains(n)) {

                                if (closed.Capacity() < n) {
                                    closed.Reserve(std::min(_capacity * ((n % _capacity) + 1U), count));
                                }
                                closed.Add(n);

                                // Create a parent node and transfer ownership of 'current' to it. Note: 'current' is now moved!
                                open.push_back({n, curr.m_GScore + static_cast<scalar_t>(1), _h(nCoord, _end) * _weight, &buf.Emplace(std::move(curr)) });
                            }
                        }
                    }
                }
                else { // SOLUTION REACHED ...

                    // Reserve space in result:
                    result.reserve(curr.m_GScore);

                    // Recurse from end node to start node, inserting into a result buffer:
                    for (const auto* temp = &curr; temp->m_Parent != nullptr; temp = static_cast<const ASNode*>(temp->m_Parent)) {
                        result.emplace_back(Utils::ToND(temp->m_Index, _maze.Size()));
                    }

                    // Reverse the result:
                    std::reverse(result.begin(), result.end());

                    break;
                }
            }

            return result;
        }

    public:

        [[maybe_unused]]
        std::vector<coord_t> Execute(const Mazes::Graph<index_t, scalar_t>& _maze, const coord_t& _start, const coord_t& _end, const coord_t& _size, scalar_t (*_h)(const coord_t&, const coord_t&), const scalar_t& _weight, size_t _capacity) const override {

            /*
             * Determine whether to solve using a linear search or constant-time
             * method based on which is more efficient given the maze's size.
             */

            constexpr size_t LMAX = 256U;

            std::vector<coord_t> result;

            const auto count = _maze.Count();

            if (count <= 32U) {
                result = SolveLinear<16U>(_maze, _start, _end, _size, _h, _weight, _capacity);
            }
            else if (count <= 64U) {
                result = SolveLinear<32U>(_maze, _start, _end, _size, _h, _weight, _capacity);
            }
            else if (count <= 128U) {
                result = SolveLinear<64U>(_maze, _start, _end, _size, _h, _weight, _capacity);
            }
            else if (count <= LMAX) {
                result = SolveLinear<LMAX / 2U>(_maze, _start, _end, _size, _h, _weight, _capacity);
            }
            else {
                result = SolveHeap(_maze, _start, _end, _size, _h, _weight, _capacity);
            }

            return result;
        }

        [[maybe_unused]]
        std::vector<coord_t> Execute(const Mazes::Grid<Kd, weight_t>& _maze, const coord_t& _start, const coord_t& _end, scalar_t (*_h)(const coord_t&, const coord_t&), const scalar_t& _weight, size_t _capacity) const override {

            /*
             * Determine whether to solve using a linear search or constant-time
             * method based on which is more efficient given the maze's size.
             */

            constexpr size_t LMAX = 256U;

            std::vector<coord_t> result;

            const auto count = _maze.Count();

            if (count <= 64U) {
                result = SolveLinear<32U>(_maze, _start, _end, _h, _weight, _capacity);
            }
            else if (count <= 128U) {
                result = SolveLinear<64U>(_maze, _start, _end, _h, _weight, _capacity);
            }
            else if (count <= LMAX) {
                result = SolveLinear<LMAX / 2U>(_maze, _start, _end, _h, _weight, _capacity);
            }
            else {
                result = SolveHeap(_maze, _start, _end, _h, _weight, _capacity);
            }

            return result;
        }
    };

} // CHDR::Solvers

#endif //CHDR_ASTAR_HPP