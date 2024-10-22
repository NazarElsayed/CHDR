/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_ASTAR_HPP
#define CHDR_ASTAR_HPP

#include <cmath>
#include <queue>

#include "../utils/Heuristics.hpp"
#include "base/ISolver.hpp"
#include "mazes/base/IMaze.hpp"
#include "mazes/Grid.hpp"
#include "types/ExistenceSet.hpp"
#include "types/StableForwardBuf.hpp"
#include "types/Heap.hpp"
#include "utils/Utils.hpp"

namespace CHDR::Solvers {

    template<typename Tm, const size_t Kd, typename Ts>
    class AStar final {

        static_assert(std::is_integral_v<Ts> || std::is_floating_point_v<Ts>, "Ts must be either an integral or floating point type");

    private:

        using coord_t = Coord<size_t, Kd>;

        struct ASNode final {

            size_t m_Index;

            Ts m_GScore;
            Ts m_FScore;

            const ASNode* m_Parent;

            [[nodiscard]] constexpr ASNode() = default;

            [[nodiscard]] constexpr ASNode(const size_t &_coord, const Ts &_gScore, const Ts &_hScore, const ASNode* const _parent) :
                m_Index(_coord),
                m_GScore(_gScore),
                m_FScore(_gScore + _hScore),
                m_Parent(std::move(_parent)) {}

            [[nodiscard]] constexpr bool operator == (const ASNode& _node) const { return m_Index == _node.m_Index; }

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

    public:

        auto Solve(const Mazes::Graph<size_t, Kd, Ts>& _maze, const size_t& _start, const size_t& _end, Ts (*_h)(const size_t&, const size_t&), const Ts& _weight = 1, size_t _capacity = 0U) const {

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

                    _capacity = std::max(_capacity, std::max(_start, _end));

                    ExistenceSet<LowMemoryUsage> closed({ _start }, _capacity);

                    Heap<ASNode, 2U, typename ASNode::Max> open(_capacity / 4U);
                    open.Emplace({ _start, static_cast<Ts>(0), _h(_start, _end), nullptr });

                    StableForwardBuf<ASNode*> buf;

                    while (!open.Empty()) {

                        auto curr = open.PopTop();

                        if (curr.m_Index != _end) { // SEARCH FOR SOLUTION...

                            if (closed.Capacity() > curr.m_Index) {
                                closed.Reserve(std::min(_capacity * ((curr.m_Index % _capacity) + 1U), maze_count));
                            }
                            closed.Add(curr.m_Index);

                            for (const auto& neighbour : _maze.GetNeighbours(curr.m_Index)) {

                                const auto& [nID, nDistance] = neighbour;

                                const auto n = nID;

                                // Check if node is not already visited:
                                if (!closed.Contains(n)) {

                                    if (closed.Capacity() > curr.m_Index) {
                                        closed.Reserve(std::min(_capacity * ((curr.m_Index % _capacity) + 1U), maze_count));
                                    }
                                    closed.Add(n);

                                    // Create a parent node and transfer ownership of 'current' to it. Note: 'current' is now moved!
                                    open.Emplace({n, curr.m_GScore + static_cast<Ts>(nDistance), _h(nID, _end) * _weight, buf.Emplace(new ASNode(std::move(curr))) });
                                }
                            }
                        }
                        else { // SOLUTION REACHED ...

                            // Recurse from end node to start node, inserting into a result buffer:
                            result.reserve(curr.m_GScore);
                            for (const auto* temp = &curr; temp->m_Parent != nullptr; temp = temp->m_Parent) {
                                result.emplace_back(temp->m_Index);
                            }

                            // Clear the buffer:
                            std::for_each(buf.begin(), buf.end(), [](auto* item) {
                                delete item;
                            });

                            // Reverse the result:
                            std::reverse(result.begin(), result.end());

                            break;
                        }
                    }
                }
            }

            return result;
        }

        auto Solve(const Mazes::Grid<Kd, Tm>& _maze, const coord_t& _start, const coord_t& _end, Ts (*_h)(const coord_t&, const coord_t&), const Ts& _weight = 1, size_t _capacity = 0U) const {

            /*
             * Determine whether to solve using a linear search or constant-time
             * method based on which is more efficient given the maze's size.
             */

            constexpr size_t h_efficiency = 256U;

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

                    ExistenceSet<LowMemoryUsage> closed({ s }, _capacity);

                    Heap<ASNode, 2U, typename ASNode::Max> open(_capacity / 4U);
                    open.Emplace({ s, static_cast<Ts>(0), _h(_start, _end), nullptr });

                    StableForwardBuf<ASNode*> buf;

                    while (!open.Empty()) {

                        auto curr = open.PopTop();

                        if (curr.m_Index != e) { // SEARCH FOR SOLUTION...

                            if (closed.Capacity() > curr.m_Index) {
                                closed.Reserve(std::min(_capacity * ((curr.m_Index % _capacity) + 1U), maze_count));
                            }
                            closed.Add(curr.m_Index);

                            for (const auto& neighbour : _maze.GetNeighbours(curr.m_Index)) {

                                if (const auto& [nActive, nCoord] = neighbour; nActive) {

                                    const auto n = Utils::To1D(nCoord, _maze.Size());

                                    // Check if node is not already visited:
                                    if (!closed.Contains(n)) {

                                        if (closed.Capacity() > curr.m_Index) {
                                            closed.Reserve(std::min(_capacity * ((curr.m_Index % _capacity) + 1U), maze_count));
                                        }
                                        closed.Add(n);

                                        // Create a parent node and transfer ownership of 'current' to it. Note: 'current' is now moved!
                                        open.Emplace({n, curr.m_GScore + static_cast<Ts>(1), _h(nCoord, _end) * _weight, buf.Emplace(new ASNode(std::move(curr))) });
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

                            // Clear the buffer:
                            std::for_each(buf.begin(), buf.end(), [](auto* item) {
                                delete item;
                            });

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

                    ExistenceSet<LowMemoryUsage> closed({ s }, _capacity);

                    std::vector<ASNode> open;
                    open.reserve(_capacity / 4U);
                    open.push_back({s, static_cast<Ts>(0), _h(_start, _end), nullptr });

                    StableForwardBuf<ASNode*, 64U> buf;

                    while (!open.empty()) {

                        const auto top = std::min_element(open.begin(), open.end(), typename ASNode::Min());
                        auto curr(std::move(*top));
                        open.erase(top);

                        if (curr.m_Index != e) { // SEARCH FOR SOLUTION...

                            if (closed.Capacity() > curr.m_Index) {
                                closed.Reserve(std::min(_capacity * ((curr.m_Index % _capacity) + 1U), maze_count));
                            }
                            closed.Add(curr.m_Index);

                            for (const auto& neighbour : _maze.GetNeighbours(curr.m_Index)) {

                                if (const auto& [nActive, nCoord] = neighbour; nActive) {

                                    const auto n = Utils::To1D(nCoord, _maze.Size());

                                    // Check if node is not already visited:
                                    if (!closed.Contains(n)) {

                                        if (closed.Capacity() > curr.m_Index) {
                                            closed.Reserve(std::min(_capacity * ((curr.m_Index % _capacity) + 1U), maze_count));
                                        }
                                        closed.Add(n);

                                        // Create a parent node and transfer ownership of 'current' to it. Note: 'current' is now moved!
                                        open.push_back({n, curr.m_GScore + static_cast<Ts>(1), _h(nCoord, _end) * _weight, buf.Emplace(new ASNode(std::move(curr))) });
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

                            // Clear the buffer:
                            std::for_each(buf.begin(), buf.end(), [](auto* item) {
                                delete item;
                            });

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

#endif //CHDR_ASTAR_HPP