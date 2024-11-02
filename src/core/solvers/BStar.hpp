/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_BSTAR_HPP
#define CHDR_BSTAR_HPP

#include "../utils/Heuristics.hpp"
#include "base/ISolver.hpp"
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
    class [[maybe_unused]] BStar final {

        static_assert(std::is_integral_v<scalar_t> || std::is_floating_point_v<scalar_t>, "scalar_t must be either an integral or floating point type.");
        static_assert(std::is_integral_v<index_t>, "index_t must be an integral type.");

    private:

        using coord_t = Coord<index_t, Kd>;

        struct BSNode final : public UnmanagedNode<index_t> {

            scalar_t m_HScore;

            /**
             * @brief Constructs an uninitialized BSNode.
             *
             * This constructor creates an BSNode with uninitialized members.
             */
            [[nodiscard]] constexpr BSNode() {} // NOLINT(*-pro-type-member-init, *-use-equals-default)

            [[nodiscard]] constexpr BSNode(const index_t& _index, const scalar_t& _hScore, const UnmanagedNode<index_t>* RESTRICT const _parent) : UnmanagedNode<index_t>(_index, _parent),
                    m_HScore(_hScore) {}

            struct Max {

                [[nodiscard]] constexpr bool operator () (const BSNode& _a, const BSNode& _b) const {
                    return _a.m_HScore > _b.m_HScore;
                }
            };

            struct Min {

                [[nodiscard]] constexpr bool operator () (const BSNode& _a, const BSNode& _b) const {
                    return _a.m_HScore < _b.m_HScore;
                }
            };
        };

        auto SolveHeap(const Mazes::Graph<index_t, scalar_t>& _maze, const coord_t& _start, const coord_t& _end, const coord_t& _size, scalar_t (*_h)(const coord_t&, const coord_t&), size_t _capacity = 0U) const {

            std::vector<coord_t> result;

            const auto s = Utils::To1D(_start, _size);
            const auto e = Utils::To1D(_end,   _size);

            if (_maze.Contains(s) &&
                _maze.Contains(e) &&
                _maze.At(s).IsActive() &&
                _maze.At(e).IsActive()
            ) {

                if (s != e) {

                    const auto count = _maze.Count();

                    _capacity = _capacity == 0U ? std::max(_maze.Count() / 10U, static_cast<size_t>(1U)) : _capacity;

                    ExistenceSet<LowMemoryUsage> closed({s }, _capacity);

                    Heap<BSNode, 2U, typename BSNode::Max> open(_capacity / 4U);
                    open.Emplace({ s, _h(_start, _end), nullptr });

                    StableForwardBuf<BSNode, 1024U * 1024U> buf;

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
                                    open.Emplace({ n, _h(CHDR::Utils::ToND(n, _size), _end), &buf.Emplace(std::move(curr)) });
                                }
                            }
                        }
                        else { // SOLUTION REACHED ...

                            // Recurse from end node to start node, inserting into a result buffer:
                            result.reserve(static_cast<size_t>(_h(_start, _end)));
                            for (const auto* temp = &curr; temp->m_Parent != nullptr; temp = static_cast<const BSNode*>(temp->m_Parent)) {
                                result.emplace_back(Utils::ToND(temp->m_Index, _size));
                            }

                            // Reverse the result:
                            std::reverse(result.begin(), result.end());

                            break;
                        }
                    }
                }
                else {
                    result.push_back(_end);
                }
            }

            return result;
        }

        template <size_t StackSize>
        auto SolveLinear(const Mazes::Graph<index_t, scalar_t>& _maze, const coord_t& _start, const coord_t& _end, const coord_t& _size, scalar_t (*_h)(const coord_t&, const coord_t&), size_t _capacity = 0U) const {

            std::vector<coord_t> result;

            const auto s = Utils::To1D(_start, _size);
            const auto e = Utils::To1D(_end,   _size);

            if (_maze.Contains(s) &&
                _maze.Contains(e) &&
                _maze.At(s).IsActive() &&
                _maze.At(e).IsActive()
            ) {

                if (s != e) {

                    const auto count = _maze.Count();

                    _capacity = std::max(_capacity, std::max(s, e));

                    ExistenceSet<LowMemoryUsage> closed({ s }, _capacity);

                    std::vector<BSNode, StackAllocator<BSNode, StackSize>> open;
                    open.reserve(StackSize);
                    open.push_back({ s, _h(_start, _end), nullptr });

                    StableForwardBuf<BSNode, StackSize / 2U> buf;

                    while (!open.empty()) {

                        const auto top = std::min_element(open.begin(), open.end(), typename BSNode::Min());
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
                                    open.push_back({ n, _h(CHDR::Utils::ToND(n, _size), _end), &buf.Emplace(std::move(curr)) });
                                }
                            }
                        }
                        else { // SOLUTION REACHED ...

                            // Recurse from end node to start node, inserting into a result buffer:
                            result.reserve(static_cast<size_t>(_h(_start, _end)));

                            for (const auto* temp = &curr; temp->m_Parent != nullptr; temp = static_cast<const BSNode*>(temp->m_Parent)) {
                                result.emplace_back(Utils::ToND(temp->m_Index, _size));
                            }

                            // Reverse the result:
                            std::reverse(result.begin(), result.end());

                            break;
                        }
                    }
                }
                else {
                    result.emplace_back(_end);
                }
            }

            return result;
        }

        auto SolveHeap(const Mazes::Grid<Kd, weight_t>& _maze, const coord_t& _start, const coord_t& _end, scalar_t (*_h)(const coord_t&, const coord_t&), size_t _capacity = 0U) const {

            std::vector<coord_t> result;

            const auto s = Utils::To1D(_start, _maze.Size());
            const auto e = Utils::To1D(_end,   _maze.Size());

            if (_maze.Contains(s) &&
                _maze.Contains(e) &&
                _maze.At(s).IsActive() &&
                _maze.At(e).IsActive()
            ) {

                if (s != e) {

                    const auto count = _maze.Count();

                    _capacity = std::max(_capacity, std::max(s, e));

                    ExistenceSet<LowMemoryUsage> closed({ s }, _capacity);

                    Heap<BSNode, 2U, typename BSNode::Max> open(_capacity / 8U);
                    open.Emplace({ s, _h(_start, _end), nullptr });

                    StableForwardBuf<BSNode> buf;

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
                                        open.Emplace({ n, _h(nCoord, _end), &buf.Emplace(std::move(curr)) });
                                    }
                                }
                            }
                        }
                        else { // SOLUTION REACHED ...

                            // Recurse from end node to start node, inserting into a result buffer:
                            result.reserve(static_cast<size_t>(_h(_start, _end)));
                            for (const auto* temp = &curr; temp->m_Parent != nullptr; temp = static_cast<const BSNode*>(temp->m_Parent)) {
                                result.emplace_back(Utils::ToND(temp->m_Index, _maze.Size()));
                            }

                            // Reverse the result:
                            std::reverse(result.begin(), result.end());

                            break;
                        }
                    }
                }
                else {
                    result.emplace_back(_end);
                }
            }

            return result;
        }

        template <size_t StackSize>
        auto SolveLinear(const Mazes::Grid<Kd, weight_t>& _maze, const coord_t& _start, const coord_t& _end, scalar_t (*_h)(const coord_t&, const coord_t&), size_t _capacity = 0U) const {

            std::vector<coord_t> result;

            const auto s = Utils::To1D(_start, _maze.Size());
            const auto e = Utils::To1D(_end,   _maze.Size());

            if (_maze.Contains(s) &&
                _maze.Contains(e) &&
                _maze.At(s).IsActive() &&
                _maze.At(e).IsActive()
            ) {

                if (s != e) {

                    const auto count = _maze.Count();

                    _capacity = std::max(_capacity, std::max(s, e));

                    ExistenceSet<LowMemoryUsage> closed({ s }, _capacity);

                    std::vector<BSNode, StackAllocator<BSNode, StackSize>> open;
                    open.reserve(StackSize);
                    open.push_back({ s, _h(_start, _end), nullptr });

                    StableForwardBuf<BSNode, StackSize / 2U> buf;

                    while (!open.empty()) {

                        const auto top = std::min_element(open.begin(), open.end(), typename BSNode::Min());
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
                                        open.push_back({ n, _h(nCoord, _end), &buf.Emplace(std::move(curr)) });
                                    }
                                }
                            }
                        }
                        else { // SOLUTION REACHED ...

                            // Recurse from end node to start node, inserting into a result buffer:
                            result.reserve(static_cast<size_t>(_h(_start, _end)));

                            for (const auto* temp = &curr; temp->m_Parent != nullptr; temp = static_cast<const BSNode*>(temp->m_Parent)) {
                                result.emplace_back(Utils::ToND(temp->m_Index, _maze.Size()));
                            }

                            // Reverse the result:
                            std::reverse(result.begin(), result.end());

                            break;
                        }
                    }
                }
                else {
                    result.emplace_back(_end);
                }
            }

            return result;
        }

    public:

        [[maybe_unused]]
        auto Solve(const Mazes::Graph<index_t, scalar_t>& _maze, const coord_t& _start, const coord_t& _end, const coord_t& _size, scalar_t (*_h)(const coord_t&, const coord_t&), size_t _capacity = 0U) const {

            /*
             * Determine whether to solve using a linear search or constant-time
             * method based on which is more efficient given the maze's size.
             */

            constexpr size_t LMAX = 256U;

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
            else if (count <= LMAX) {
                result = SolveLinear<LMAX / 2U>(_maze, _start, _end, _size, _h, _capacity);
            }
            else {
                result = SolveHeap(_maze, _start, _end, _size, _h, _capacity);
            }

            return result;
        }

        [[maybe_unused]]
        auto Solve(const Mazes::Grid<Kd, weight_t>& _maze, const coord_t& _start, const coord_t& _end, scalar_t (*_h)(const coord_t&, const coord_t&), size_t _capacity = 0U) const {

            /*
             * Determine whether to solve using a linear search or constant-time
             * method based on which is more efficient given the maze's size.
             */

            constexpr size_t LMAX = 256U;

            std::vector<coord_t> result;

            const auto count = _maze.Count();

            if (count <= 64U) {
                result = SolveLinear<32U>(_maze, _start, _end, _h, _capacity);
            }
            else if (count <= 128U) {
                result = SolveLinear<64U>(_maze, _start, _end, _h, _capacity);
            }
            else if (count <= LMAX) {
                result = SolveLinear<LMAX / 2U>(_maze, _start, _end, _h, _capacity);
            }
            else {
                result = SolveHeap(_maze, _start, _end, _h, _capacity);
            }

            return result;
        }
    };

} // CHDR::Solvers

#endif //CHDR_BSTAR_HPP