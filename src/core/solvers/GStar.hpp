/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_GSTAR_HPP
#define CHDR_GSTAR_HPP

#include <memory>
#include <queue>

#include "base/ISolver.hpp"
#include "mazes/base/IMaze.hpp"
#include "types/ExistenceSet.hpp"
#include "types/Heap.hpp"
#include "utils/Utils.hpp"
#include "mazes/Grid.hpp"

namespace CHDR::Solvers {

    template<typename Tm, const size_t Kd, typename Ts>
    class GStar final {

        static_assert(std::is_integral_v<Ts> || std::is_floating_point_v<Ts>, "Ts must be either an integral or floating point type");

    private:

        using coord_t = Coord<size_t, Kd>;

        struct GSNode final : IHeapItem {

            size_t m_Index;

            Ts m_GScore;
            Ts m_FScore;

            std::shared_ptr<const GSNode> m_Parent;

            [[nodiscard]] constexpr GSNode(const size_t &_coord, const Ts &_gScore, const Ts &_hScore, const std::shared_ptr<const GSNode>& _parent) : IHeapItem(),
                m_Index(_coord),
                m_GScore(_gScore),
                m_FScore(_gScore + _hScore),
                m_Parent(std::move(_parent)) {}

            ~GSNode() {

                while (m_Parent && static_cast<unsigned>(m_Parent.use_count()) < 2U) {
                    m_Parent = std::move(m_Parent->m_Parent);
                }
            }

            [[nodiscard]] constexpr bool operator == (const GSNode& _node) const { return m_Index == _node.m_Index; }

            struct Max {

                [[nodiscard]] constexpr bool operator () (const GSNode& _a, const GSNode& _b) const {

                    return _a.m_FScore == _b.m_FScore ?
                        _a.m_GScore > _b.m_GScore :
                        _a.m_FScore > _b.m_FScore;
                }
            };

            struct Min {

                [[nodiscard]] constexpr bool operator () (const GSNode& _a, const GSNode& _b) const {

                    return _a.m_FScore == _b.m_FScore ?
                        _a.m_GScore < _b.m_GScore :
                        _a.m_FScore < _b.m_FScore;
                }
            };
        };

    public:

        auto Solve(const Mazes::Grid<Kd, Tm>& _maze, const coord_t& _start, const coord_t& _end, Ts (*_h)(const coord_t&, const coord_t&), const Ts& _weight = 1, size_t _capacity = 0U) const {

            /*
             * Determine whether to solve using a linear search or constant-time
             * method based on which is more efficient given the maze's size.
             */

            constexpr size_t h_efficiency = 361U;

            return _maze.Count() >= h_efficiency ?
                SolveHeap   (_maze, _start, _end, _h, _weight, _capacity) :
                SolveLinear (_maze, _start, _end, _h, _weight, _capacity);
        }

        auto SolveHeap(const Mazes::Grid<Kd, Tm>& _maze, const coord_t& _start, const coord_t& _end, Ts (*_h)(const coord_t&, const coord_t&), const Ts& _weight = 1, size_t _capacity = 0U) const {

            std::vector<coord_t> result;

            const auto s = Utils::To1D(_start, _maze.Size());
            const auto e = Utils::To1D(_end,   _maze.Size());

            if (_maze.Contains(s) &&
                _maze.Contains(e) &&
                _maze.At(s).IsActive() &&
                _maze.At(e).IsActive()
            ) {

                if (s != e) {

                    const auto maze_count = _maze.Count();

                    _capacity = std::max(_capacity, std::max(s, e));

                    ExistenceSet closedSet({ s }, _capacity);

                    Heap<GSNode, typename GSNode::Max> openSet(_capacity / 4U);
                    openSet.Emplace({ s, static_cast<Ts>(0), _h(_start, _end), nullptr });

                    while (!openSet.Empty()) {

                        GSNode current(std::move(openSet.Top()));
                        openSet.RemoveFirst();

                        if (current.m_Index != e) { // SEARCH FOR SOLUTION...

                            if (closedSet.Capacity() > current.m_Index) {
                                closedSet.Reserve(std::min(_capacity * ((current.m_Index % _capacity) + 1U), maze_count));
                            }
                            closedSet.Add(current.m_Index);

                            for (const auto& neighbour : _maze.GetNeighbours(current.m_Index)) {

                                if (const auto& [nActive, nCoord] = neighbour; nActive) {

                                    const auto n = Utils::To1D(nCoord, _maze.Size());

                                    // Check if node is not already visited:
                                    if (!closedSet.Contains(n)) {

                                        // Add to dupe list:
                                        if (closedSet.Capacity() > n) {
                                            closedSet.Reserve(std::min(_capacity * ((n % _capacity) + 1U), maze_count));
                                        }
                                        closedSet.Add(n);

                                        // Create a parent node and transfer ownership of 'current' to it. Note: 'current' is now moved!
                                        openSet.Emplace({ n, current.m_GScore + static_cast<Ts>(1), _h(nCoord, _end) * _weight, std::make_shared<GSNode>(std::move(current)) });
                                    }
                                }
                            }
                        }
                        else { // SOLUTION REACHED ...

                            // Free data which is no longer relevant:
                              openSet.Clear();
                            closedSet.Clear();

                            // Recurse from end node to start node, inserting into a result buffer:
                            result.reserve(current.m_GScore);
                            result.emplace_back(Utils::ToND(current.m_Index, _maze.Size()));

                            if (current.m_Parent != nullptr) {

                                for (auto& item = current.m_Parent; item->m_Parent != nullptr;) {
                                    result.emplace_back(Utils::ToND(item->m_Index, _maze.Size()));

                                    auto oldItem = item;
                                    item = item->m_Parent;
                                    oldItem.reset();
                                }
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

        auto SolveLinear(const Mazes::Grid<Kd, Tm>& _maze, const coord_t& _start, const coord_t& _end, Ts (*_h)(const coord_t&, const coord_t&), const Ts& _weight = 1, size_t _capacity = 0U) const {

            std::vector<coord_t> result;

            const auto s = Utils::To1D(_start, _maze.Size());
            const auto e = Utils::To1D(_end,   _maze.Size());

            if (_maze.Contains(s) &&
                _maze.Contains(e) &&
                _maze.At(s).IsActive() &&
                _maze.At(e).IsActive()
            ) {

                if (s != e) {

                    const auto maze_count = _maze.Count();

                    _capacity = std::max(_capacity, std::max(s, e));

                    ExistenceSet closedSet({ s }, _capacity);

                    std::vector<GSNode> openSet;
                    openSet.reserve(_capacity / 4U);
                    openSet.push_back({ s, static_cast<Ts>(0), _h(_start, _end), nullptr });

                    while (!openSet.empty()) {

                        const auto top = std::min_element(openSet.begin(), openSet.end(), typename GSNode::Min()); // Linear search

                        GSNode current(std::move(*top));
                        openSet.erase(top);

                        if (current.m_Index != e) { // SEARCH FOR SOLUTION...

                            if (closedSet.Capacity() > current.m_Index) {
                                closedSet.Reserve(std::min(_capacity * ((current.m_Index % _capacity) + 1U), maze_count));
                            }
                            closedSet.Add(current.m_Index);

                            for (const auto& neighbour : _maze.GetNeighbours(current.m_Index)) {

                                if (const auto& [nActive, nCoord] = neighbour; nActive) {

                                    const auto n = Utils::To1D(nCoord, _maze.Size());

                                    // Check if node is not already visited:
                                    if (!closedSet.Contains(n)) {

                                        // Add to dupe list:
                                        if (closedSet.Capacity() > n) {
                                            closedSet.Reserve(std::min(_capacity * ((n % _capacity) + 1U), maze_count));
                                        }
                                        closedSet.Add(n);

                                        // Create a parent node and transfer ownership of 'current' to it. Note: 'current' is now moved!
                                        openSet.push_back({ n, current.m_GScore + static_cast<Ts>(1), _h(nCoord, _end) * _weight, std::make_shared<GSNode>(std::move(current)) });
                                    }
                                }
                            }
                        }
                        else { // SOLUTION REACHED ...

                            // Free data which is no longer relevant:
                              openSet.clear();   openSet.Trim();
                            closedSet.Clear(); closedSet.Trim();

                            // Recurse from end node to start node, inserting into a result buffer:
                            result.reserve(current.m_GScore);
                            result.emplace_back(Utils::ToND(current.m_Index, _maze.Size()));

                            if (current.m_Parent != nullptr) {

                                for (auto& item = current.m_Parent; item->m_Parent != nullptr;) {
                                    result.emplace_back(Utils::ToND(item->m_Index, _maze.Size()));

                                    auto oldItem = item;
                                    item = item->m_Parent;
                                    oldItem.reset();
                                }
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

    };

} // CHDR::Solvers

#endif //CHDR_GSTAR_HPP