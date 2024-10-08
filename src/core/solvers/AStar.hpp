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
#include "types/Heap.hpp"
#include "utils/Utils.hpp"

namespace CHDR::Solvers {

    template<typename Tm, const size_t Kd, typename Ts>
    class AStar final {

        static_assert(std::is_integral_v<Ts> || std::is_floating_point_v<Ts>, "Ts must be either an integral or floating point type");

    private:

        using coord_t = Coord<size_t, Kd>;

        struct ASNode final : IHeapItem {

            size_t m_Index;

            Ts m_GScore;
            Ts m_FScore;

            const ASNode* m_Parent;

            [[nodiscard]] constexpr ASNode(const size_t &_coord, const Ts &_gScore, const Ts &_hScore, const ASNode* const _parent) : IHeapItem(),
                m_Index(_coord),
                m_GScore(_gScore),
                m_FScore(_gScore + _hScore),
                m_Parent(std::move(_parent)) {}

            [[nodiscard]] constexpr bool operator == (const ASNode& _node) const { return m_Index == _node.m_Index; }

            struct Max {

                [[nodiscard]] constexpr bool operator () (const ASNode& _a, const ASNode& _b) const {

                    bool result{};

                    if (_a.m_FScore == _b.m_FScore) {
                        result = _a.m_GScore > _b.m_GScore;
                    }
                    else {
                        result = _a.m_FScore > _b.m_FScore;
                    }

                    return result;
                }
            };
        };

    public:

        auto Solve(const Mazes::Graph<size_t, Kd, Ts>& _maze, const size_t& _start, const size_t& _end, Ts (*_h)(const size_t&, const size_t&), size_t _capacity = 0U) const {

            std::vector<size_t> result;

            _capacity = std::max(_capacity, std::max(_start, _end));

            ExistenceSet closedSet({ _start }, _capacity);

            Heap<ASNode, typename ASNode::Max> openSet;
            openSet.Emplace({ _start, static_cast<Ts>(0), _h(_start, _end), nullptr });

            std::vector<ASNode*> buffer;

            while (!openSet.Empty()) {

                ASNode current(std::move(openSet.Top()));
                openSet.RemoveFirst();

                if (current.m_Index != _end) { // SEARCH FOR SOLUTION...

                    if (closedSet.Capacity() > current.m_Index) {
                        closedSet.Reserve(std::min(_capacity * ((current.m_Index % _capacity) + 1U), _maze.Count()));
                    }
                    closedSet.Add(current.m_Index);

                    for (const auto& neighbour : _maze.GetNeighbours(current.m_Index)) {

                        const auto& [nID, nDistance] = neighbour;

                        const auto n = nID;

                        // Check if node is not already visited:
                        if (!closedSet.Contains(n)) {

                            if (closedSet.Capacity() > current.m_Index) {
                                closedSet.Reserve(std::min(_capacity * ((current.m_Index % _capacity) + 1U), _maze.Count()));
                            }
                            closedSet.Add(n);

                            // Create a parent node and transfer ownership of 'current' to it. Note: 'current' is now moved!
                            buffer.emplace_back(new ASNode(std::move(current)));
                            openSet.Emplace({ n, current.m_GScore + static_cast<Ts>(nDistance), _h(nID, _end), buffer.back() });
                        }
                    }
                }
                else { // SOLUTION REACHED ...

                    // Free data which is no longer relevant:
                      openSet.Clear();
                    closedSet.Clear();

                    // Recurse from end node to start node, inserting into a result buffer:
                    result.reserve(current.m_GScore);
                    for (const auto* temp = &current; temp->m_Parent != nullptr; temp = temp->m_Parent) {
                        result.emplace_back(temp->m_Index);
                    }

                    // Clear the buffer:
                    for (auto* item : buffer) {

                        if (item != nullptr) {
                            delete item;
                            item = nullptr;
                        }
                    }
                    buffer.clear();

                    // Reverse the result:
                    std::reverse(result.begin(), result.end());

                    break;
                }
            }

            return result;
        }

        auto Solve(const Mazes::Grid<Kd, Tm>& _maze, const coord_t& _start, const coord_t& _end, Ts (*_h)(const coord_t&, const coord_t&), size_t _capacity = 0U) const {

            std::vector<coord_t> result;

            const auto s = Utils::To1D(_start, _maze.Size());
            const auto e = Utils::To1D(_end,   _maze.Size());

            _capacity = std::max(_capacity, std::max(s, e));

            ExistenceSet closedSet({ s }, _capacity);

            Heap<ASNode, typename ASNode::Max> openSet;
            openSet.Emplace({ s, static_cast<Ts>(0), _h(_start, _end), nullptr });

            std::vector<ASNode*> buffer;

            while (!openSet.Empty()) {

                ASNode current(std::move(openSet.Top()));
                openSet.RemoveFirst();

                if (current.m_Index != e) { // SEARCH FOR SOLUTION...

                    if (closedSet.Capacity() > current.m_Index) {
                        closedSet.Reserve(std::min(_capacity * ((current.m_Index % _capacity) + 1U), _maze.Count()));
                    }
                    closedSet.Add(current.m_Index);

                    for (const auto& neighbour : _maze.GetNeighbours(current.m_Index)) {

                        if (const auto& [nActive, nCoord] = neighbour; nActive) {

                            const auto n = Utils::To1D(nCoord, _maze.Size());

                            // Check if node is not already visited:
                            if (!closedSet.Contains(n)) {

                                if (closedSet.Capacity() > current.m_Index) {
                                    closedSet.Reserve(std::min(_capacity * ((current.m_Index % _capacity) + 1U), _maze.Count()));
                                }
                                closedSet.Add(n);

                                // Create a parent node and transfer ownership of 'current' to it. Note: 'current' is now moved!
                                buffer.emplace_back(new ASNode(std::move(current)));
                                openSet.Emplace({ n, current.m_GScore + static_cast<Ts>(1), _h(nCoord, _end), buffer.back() });
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
                    for (const auto* temp = &current; temp->m_Parent != nullptr; temp = temp->m_Parent) {
                        result.emplace_back(Utils::ToND(temp->m_Index, _maze.Size()));
                    }

                    // Clear the buffer:
                    for (auto* item : buffer) {

                        if (item != nullptr) {
                            delete item;
                            item = nullptr;
                        }
                    }
                    buffer.clear();

                    // Reverse the result:
                    std::reverse(result.begin(), result.end());

                    break;
                }
            }

            return result;
        }

    };

} // CHDR::Solvers

#endif //CHDR_ASTAR_HPP