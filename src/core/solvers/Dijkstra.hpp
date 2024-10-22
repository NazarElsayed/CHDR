/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_DIJKSTRA_HPP
#define CHDR_DIJKSTRA_HPP

#include <memory>

#include "base/ISolver.hpp"
#include "mazes/base/IMaze.hpp"
#include "types/Heap.hpp"
#include "mazes/Grid.hpp"

namespace CHDR::Solvers {

    template<typename Tm, const size_t Kd, typename Ts>
    class Dijkstra final {

    private:

        using coord_t = Coord<size_t, Kd>;

        struct DijkstraNode final : IHeapItem {

            size_t m_Coord;

            Ts m_GScore;
            Ts m_FScore;

            std::shared_ptr<const DijkstraNode> m_Parent;

            [[nodiscard]] constexpr DijkstraNode(const size_t &_coord, const Ts &_gScore, const Ts &_hScore, const std::shared_ptr<const DijkstraNode>& _parent) : IHeapItem(),
                m_Coord(_coord),
                m_GScore(_gScore),
                m_FScore(_gScore + _hScore),
                m_Parent(std::move(_parent)) {}

            ~DijkstraNode() {

                while (m_Parent && static_cast<unsigned>(m_Parent.use_count()) < 2U) {
                    m_Parent = std::move(m_Parent->m_Parent);
                }
            }

            [[nodiscard]] constexpr bool operator == (const DijkstraNode& _node) const { return m_Coord == _node.m_Coord; }

            struct Max {

                [[nodiscard]] constexpr bool operator () (const DijkstraNode& _a, const DijkstraNode& _b) const {

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

        void Solve(const Mazes::Grid<Kd, Tm>& _maze, const coord_t& _start, const coord_t& _end, Ts (*_h)(const coord_t&, const coord_t&), size_t _capacity = 0U) {

            (void)_maze; // Suppress unused variable warnings.
            (void)_start;
            (void)_end;
            (void)_h;
            (void)_capacity;

            throw std::runtime_error("Djikstra::Solve(const Mazes::Grid<Kd, Tm>& _maze, const coord_t& _start, const coord_t& _end, Ts (*_h)(const coord_t&, const coord_t&), size_t _capacity = 0U): Not implemented!");

//
//            std::vector<coord_t> result;
//
//            const auto s = Utils::To1D(_start, _maze.Size());
//            const auto e = Utils::To1D(_end,   _maze.Size());
//
//            _capacity = std::max(_capacity, std::max(s, e));
//
//            ExistenceSet closedSet ({ s }, _capacity);
//
//            Heap<DijkstraNode, typename DijkstraNode::Max> openSet(_capacity / 4U);
//            openSet.Emplace({ s, static_cast<Ts>(0), _h(_start, _end), nullptr });
//
//            while (!openSet.Empty()) {
//
//                DijkstraNode current(std::move(openSet.Top()));
//                openSet.RemoveFirst();
//
//                if (current.m_Coord != e) { // SEARCH FOR SOLUTION...
//
//                    if (closedSet.Capacity() > current.m_Coord) {
//                        closedSet.Reserve(std::min(_capacity * ((current.m_Coord % _capacity) + 1U), maze_count));
//                    }
//                    closedSet.Add(current.m_Coord);
//
//                    for (const auto& neighbour : _maze.GetNeighbours(current.m_Coord)) {
//
//                        if (const auto& [nActive, nCoord] = neighbour; nActive) {
//
//                            const auto n = Utils::To1D(nCoord, _maze.Size());
//
//                            // Check if node is not already visited:
//                            if (!closedSet.Contains(n)) {
//
//                                // Add to dupe list:
//                                if (closedSet.Capacity() > n) {
//                                    closedSet.Reserve(std::min(_capacity * ((n % _capacity) + 1U), maze_count));
//                                }
//                                closedSet.Add(n);
//
//                                // Create a parent node and transfer ownership of 'current' to it. Note: 'current' is now moved!
//                                openSet.Emplace({ n, current.m_GScore + static_cast<Ts>(1), _h(nCoord, _end), std::make_shared<DijkstraNode>(std::move(current)) });
//                            }
//                        }
//                    }
//                }
//                else { // SOLUTION REACHED ...
//
//                    // Recurse from end node to start node, inserting into a result buffer:
//                    result.reserve(current.m_GScore);
//                    result.emplace_back(Utils::ToND(current.m_Coord, _maze.Size()));
//
//                    if (current.m_Parent != nullptr) {
//
//                        for (auto& item = current.m_Parent; item->m_Parent != nullptr;) {
//                            result.emplace_back(Utils::ToND(item->m_Coord, _maze.Size()));
//
//                            auto oldItem = item;
//                            item = item->m_Parent;
//                            oldItem.reset();
//                        }
//                    }
//
//                    // Reverse the result:
//                    std::reverse(result.begin(), result.end());
//
//                    break;
//                }
//            }
//
//            return result;
        }

    };

} // CHDR::Solvers

#endif //CHDR_DJIKSTRA_HPP