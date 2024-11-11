/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_JPS_HPP
#define CHDR_JPS_HPP

#include <map>

#include "base/BSolver.hpp"
#include "mazes/Grid.hpp"
#include "types/ExistenceSet.hpp"
#include "types/StableForwardBuf.hpp"
#include "utils/Utils.hpp"

namespace CHDR::Solvers {

    template<typename weight_t, const size_t Kd, typename scalar_t, typename index_t>
    class [[maybe_unused]] JPS final : public BSolver<weight_t, Kd, scalar_t, index_t> {

        static_assert(std::is_integral_v<scalar_t> || std::is_floating_point_v<scalar_t>, "scalar_t must be either an integral or floating point type");
        static_assert(std::is_integral_v<index_t>, "index_t must be an integral type.");

    private:

        using coord_t = Coord<index_t, Kd>;

        static constexpr std::array<uint8_t, 8U> s_rotateL { 2U, 4U, 7U,
                                                             1U,     6U,
                                                             0U, 3U, 5U };

        static constexpr std::array<uint8_t, 8U> s_rotate2 { 7U, 6U, 5U,
                                                             4U,     3U,
                                                             2U, 1U, 0U };

        static constexpr std::array<uint8_t, 8U> s_rotateR { 5U, 3U, 0U,
                                                             6U,     1U,
                                                             7U, 4U, 2U };

        const std::map<std::array<int8_t, 2U>, std::array<uint8_t, 8U>> rotationMap {
                { { 0,  0}, { 0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U } },
                { { 1,  0}, { 0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U } },
                { { 1,  1}, { 0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U } },
                { { 0,  1}, s_rotateL },
                { {-1,  1}, s_rotateL },
                { {-1,  0}, s_rotate2 },
                { {-1, -1}, s_rotate2 },
                { { 0, -1}, s_rotateR },
                { { 1, -1}, s_rotateR }
        };

        struct JPSNode final : public UnmanagedNode<index_t> {

            std::array<int8_t, 2U> m_Direction;

            scalar_t m_GScore;
            scalar_t m_FScore;

            [[nodiscard]] constexpr JPSNode() : UnmanagedNode<index_t>() {};

            [[nodiscard]] constexpr JPSNode(const index_t& _index, const std::array<int8_t, 2U>& _direction, const scalar_t& _gScore, const scalar_t& _hScore, const JPSNode* RESTRICT const _parent) : UnmanagedNode<index_t>(_index, _parent),
                m_Direction(_direction),
                m_GScore(_gScore),
                m_FScore(_gScore + _hScore) {}

            struct Max {

                [[nodiscard]] constexpr bool operator () (const JPSNode& _a, const JPSNode& _b) const {

                    return _a.m_FScore == _b.m_FScore ?
                        _a.m_GScore > _b.m_GScore :
                        _a.m_FScore > _b.m_FScore;
                }
            };

            struct Min {

                [[nodiscard]] constexpr bool operator () (const JPSNode& _a, const JPSNode& _b) const {

                    return _a.m_FScore == _b.m_FScore ?
                        _a.m_GScore < _b.m_GScore :
                        _a.m_FScore < _b.m_FScore;
                }
            };
        };

        template <typename T>
        static constexpr int Sign(const T _val) {
            return (static_cast<T>(0) < _val) - (_val < static_cast<T>(0));
        }

        std::vector<coord_t> FindJumpPoints(const Mazes::Grid<Kd, weight_t>& _maze, const coord_t& _current, const std::array<int8_t, 2U> _direction, const coord_t& _end) const {

            std::vector<coord_t> result;

            const auto neighbours = _maze. template GetNeighbours<true>(_current);
            const auto map = rotationMap.at(_direction);

            // Start Node:
            if (_direction[0] == 0 && _direction[1] == 0) {
                for (auto& neighbour : neighbours) {
                    if (neighbour.first) {
                        if (const auto& [nActive, nCoord] = Jump(_maze, neighbour.second, _current, _end); nActive) {
                            result.emplace_back(nCoord);
                        }
                    }
                }
            }
            // Straight Direction:
            else if (_direction[0] == 0 || _direction[1] == 0) {

                // Check and expand forced neighbours:
                if (neighbours[map[2]].first && !neighbours[map[1]].first) {
                    if (const auto& [nActive, nCoord] = Jump(_maze, neighbours[map[2]].second, _current, _end); nActive) {
                        result.emplace_back(nCoord);
                    }
                }
                if (neighbours[map[7]].first && !neighbours[map[6]].first) {
                    if (const auto& [nActive, nCoord] = Jump(_maze, neighbours[map[7]].second, _current, _end); nActive) {
                        result.emplace_back(nCoord);
                    }
                }

                // Expand natural neighbours:
                if (neighbours[map[4]].first) {
                    if (const auto& [nActive, nCoord] = Jump(_maze, neighbours[map[4]].second, _direction, _end); nActive) {
                        result.emplace_back(nCoord);
                    }
                }
            }
            // Diagonal Direction:
            else {

                // Check diagonal is not blocked:
                if (neighbours[map[1]].first ||
                    neighbours[map[3]].first) {

                    // Check and expand forced neighbours:
                    if (neighbours[map[2]].first && !neighbours[map[1]].first) {
                        if (const auto& [nActive, nCoord] = Jump(_maze, neighbours[map[2]].second, _current, _end); nActive) {
                            result.emplace_back(nCoord);
                        }
                    }
                    if (neighbours[map[5]].first && !neighbours[map[3]].first) {
                        if (const auto& [nActive, nCoord] = Jump(_maze, neighbours[map[5]].second, _current, _end); nActive) {
                            result.emplace_back(nCoord);
                        }
                    }

                    // Expand natural neighbours:
                    if (neighbours[map[4]].first) {
                        if (const auto& [nActive, nCoord] = Jump(_maze, neighbours[map[4]].second, _current, _end); nActive) {
                            result.emplace_back(nCoord);
                        }
                    }

                    if (neighbours[map[6]].first) {
                        if (const auto& [nActive, nCoord] = Jump(_maze, neighbours[map[6]].second, _current, _end); nActive) {
                            result.emplace_back(nCoord);
                        }
                    }

                    if (neighbours[map[7]].first) {
                        if (const auto& [nActive, nCoord] = Jump(_maze, neighbours[map[7]].second, _direction, _end); nActive) {
                            result.emplace_back(nCoord);
                        }
                    }
                }
            }

            return result;
        }

        [[nodiscard]]
        std::pair<bool, coord_t> Jump(const Mazes::Grid<Kd, weight_t>& _maze, const coord_t& _current, const coord_t& _previous, const coord_t& _end) const {

            const std::array<int8_t, 2> direction { Sign(static_cast<int>(_current[0]) - static_cast<int>(_previous[0])) ,
                                                    Sign(static_cast<int>(_current[1]) - static_cast<int>(_previous[1])) };

            return Jump(_maze, _current, direction, _end);
        }

        [[nodiscard]]
        std::pair<bool, coord_t> Jump(const Mazes::Grid<Kd, weight_t>& _maze, const coord_t& _current, const std::array<int8_t, 2U>& _direction, const coord_t& _end) const {

            std::pair<bool, coord_t> result { false, _current };

            const auto neighbours = _maze. template GetNeighbours<true>(_current);
            const auto map = rotationMap.at(_direction);

            if (_direction[0] == 0 || _direction[1] == 0) { // Straight Direction

                if (_current == _end) {
                    result.first = true;
                }
                else {

                    // Check for forced neighbours:
                    if ((neighbours[map[2]].first && !neighbours[map[1]].first) ||
                        (neighbours[map[7]].first && !neighbours[map[6]].first)) {

                        result.first = true;
                    }
                    // Expand natural neighbours:
                    else {
                        if (neighbours[map[4]].first) {
                            result = Jump(_maze, neighbours[map[4]].second, _direction, _end);
                        }
                    }
                }
            }
            else { // Diagonal Direction

                // Check diagonal is not blocked:
                if (neighbours[map[1]].first ||
                    neighbours[map[3]].first) {

                    if (_current == _end) {
                        result.first = true;
                    }
                    else {

                        // Check for forced neighbours:
                        if ((neighbours[map[2]].first && !neighbours[map[1]].first) ||
                            (neighbours[map[5]].first && !neighbours[map[3]].first)) {

                            result.first = true;
                        }
                        // Expand natural neighbours:
                        else {
                            if (neighbours[map[4]].first) {
                                result.first = Jump(_maze, neighbours[map[4]].second, _current, _end).first;
                            }

                            if (!result.first) {
                                if (neighbours[map[6]].first) {
                                    result.first = Jump(_maze, neighbours[map[6]].second, _current, _end).first;
                                }
                            }

                            if (!result.first) {
                                if (neighbours[map[7]].first) {
                                    result = Jump(_maze, neighbours[map[7]].second, _direction, _end);
                                }
                            }
                        }
                    }
                }
            }

            return result;
        }

        std::vector<coord_t> Execute(const Mazes::Grid<Kd, weight_t>& _maze, const coord_t& _start, const coord_t& _end, scalar_t (*_h)(const coord_t&, const coord_t&), const scalar_t& _weight, size_t _capacity) const override {

            std::vector<coord_t> result;

            const auto s = Utils::To1D(_start, _maze.Size());
            const auto e = Utils::To1D(_end  , _maze.Size());

            // Create closed set:
            _capacity = std::max(_capacity, std::max(s, e));
            ExistenceSet<LowMemoryUsage> closed({ s }, _capacity);

            // Create open set:
            Heap<JPSNode, 2U, typename JPSNode::Max> open(_capacity / 8U);
            open.Emplace({ s, {0, 0}, static_cast<scalar_t>(0), _h(_start, _end), nullptr });

            // Create buffer:
            StableForwardBuf<JPSNode> buf;

            // Main loop:
            while (!open.Empty()) {

                auto curr = open.PopTop();

                if (curr.m_Index != e) { // SEARCH FOR SOLUTION...

                    if (closed.Capacity() < curr.m_Index) {
                        closed.Reserve(std::min(_capacity * ((curr.m_Index % _capacity) + 1U), _maze.Count()));
                    }
                    closed.Add(curr.m_Index);

                    auto coord = Utils::ToND(curr.m_Index, _maze.Size());
                    auto successors = FindJumpPoints(_maze, coord, curr.m_Direction, _end);

                    for (const auto& successor : successors) {

                        const auto n = Utils::To1D(successor, _maze.Size());
                        if (!closed.Contains(n)) {

                            // Check if node is not already visited:
                            if (closed.Capacity() < n) {
                                closed.Reserve(std::min(_capacity * ((n % _capacity) + 1U), _maze.Count()));
                            }

                            closed.Add(n);

                            const std::array<int8_t, 2> direction { Sign(static_cast<int>(successor[0]) - static_cast<int>(coord[0])) ,
                                                                    Sign(static_cast<int>(successor[1]) - static_cast<int>(coord[1])) };

                            // Create a parent node and transfer ownership of 'current' to it. Note: 'current' is now moved!
                            open.Emplace({ n, direction, curr.m_GScore + static_cast<scalar_t>(1), _h(successor, _end) * _weight, &buf.Emplace(std::move(curr)) });
                        }
                    }
                }
                else { // SOLUTION REACHED ...

                    // Reserve space in result:
                    result.reserve(curr.m_GScore);

                    // Recurse from end node to start node, inserting into a result buffer:
                    for (const auto* temp = &curr; temp->m_Parent != nullptr; temp = static_cast<const JPSNode*>(temp->m_Parent)) {
                        result.emplace_back(Utils::ToND(temp->m_Index, _maze.Size()));
                    }

                    // Reverse the result:
                    std::reverse(result.begin(), result.end());

                    break;
                }
            }

            return result;
        }

        /*template <size_t StackSize>
        auto SolveLinear(const Mazes::Grid<Kd, weight_t>& _maze, const coord_t& _start, const coord_t& _end, scalar_t (*_h)(const coord_t&, const coord_t&), const scalar_t& _weight = 1, size_t _capacity = 0U) const {

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

                    std::vector<JPSNode, StackAllocator<JPSNode, StackSize>> open;
                    open.reserve(StackSize);
                    open.push_back({ s, static_cast<scalar_t>(0), _h(_start, _end), nullptr });

                    StableForwardBuf<JPSNode, StackSize / 2U> buf;

                    while (!open.empty()) {

                        const auto top = std::min_element(open.begin(), open.end(), typename JPSNode::Min());
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

                            // Recurse from end node to start node, inserting into a result buffer:
                            result.reserve(curr.m_GScore);

                            for (const auto* temp = &curr; temp->m_Parent != nullptr; temp = temp->m_Parent) {
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
        }*/

    public:

        /*[[maybe_unused]]
        auto Solve(const Mazes::Grid<Kd, weight_t>& _maze, const coord_t& _start, const coord_t& _end, scalar_t (*_h)(const coord_t&, const coord_t&), const scalar_t& _weight = 1, size_t _capacity = 0U) const {
            /*
             * Determine whether to solve using a linear search or constant-time
             * method based on which is more efficient given the maze's size.
             #1#

            std::vector<coord_t> result;

            /*
            constexpr size_t LMAX = 256U;

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
            #1#

            result = SolveHeap(_maze, _start, _end, _h, _weight, _capacity);

            return result;
        }*/
    };

} // CHDR::Solvers

#endif //CHDR_JPS_HPP