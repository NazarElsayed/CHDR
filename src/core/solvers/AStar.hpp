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

#include <Debug.hpp>

#include <functional>
#include <memory>
#include <queue>
#include <unordered_set>
#include <list>
#include <future>

#include "base/ISolver.hpp"
#include "types/Heap.hpp"
#include "types/DenseExistenceSet.hpp"

#include "../utils/Heuristics.hpp"

namespace CHDR::Solvers {

    template<typename Tm, const size_t Kd, typename Ts>
    class AStar final : public ISolver<Tm> {

        static_assert(std::is_integral_v<Ts> || std::is_floating_point_v<Ts>, "Ts must be either an integral or floating point type");

    private:

        using coord_t = Coord<size_t, Kd>;

        struct ASNode final : IHeapItem {

            size_t m_Coord;

            Ts m_GScore;
            Ts m_FScore;

            std::shared_ptr<const ASNode> m_Parent;

            [[nodiscard]] constexpr ASNode(const size_t &_coord, const Ts &_gScore, const Ts &_hScore, const std::shared_ptr<const ASNode>& _parent) : IHeapItem(),
                m_Coord(_coord),
                m_GScore(_gScore),
                m_FScore(_gScore + _hScore),
                m_Parent(std::move(_parent)) {}

            ~ASNode() {

                while (m_Parent && static_cast<unsigned>(m_Parent.use_count()) < 2U) {
                    m_Parent = std::move(m_Parent->m_Parent);
                }
            }

            [[nodiscard]] constexpr bool operator == (const ASNode& _node) const { return m_Coord == _node.m_Coord; }

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

        void Solve(const Mazes::IMaze<Tm>& _maze) override {

            (void)_maze; // Suppress unused variable warning.

            throw std::runtime_error("AStar::Solve(const Mazes::IMaze& _maze) Not implemented!");
        }

        auto Solve(const Mazes::Grid<Kd, Tm>& _maze, const coord_t& _start, const coord_t& _end, Ts (*_h)(const coord_t&, const coord_t&), size_t _capacity = 0U) const {

            std::vector<coord_t> result;

            const auto s = Utils::To1D(_start, _maze.Size());
            const auto e = Utils::To1D(_end,   _maze.Size());

            _capacity = std::max(_capacity, std::max(s, e));

            DenseExistenceSet closedSet ({ s }, _capacity);
            DenseExistenceSet dupes     (       _capacity);

            Heap<ASNode, typename ASNode::Max> openSet;
            openSet.Emplace({ s, static_cast<Ts>(0), _h(_start, _end), nullptr });

            while (!openSet.Empty()) {

                ASNode current(std::move(openSet.Top()));
                openSet.RemoveFirst();
                dupes.Remove(current.m_Coord);

                if (current.m_Coord != e) { // SEARCH FOR SOLUTION...

                    while (closedSet.Capacity() <= current.m_Coord) {
                        closedSet.Reserve(std::min(closedSet.Capacity() * 2U, Utils::Product<size_t>(_maze.Size())));
                    }
                    closedSet.Add(current.m_Coord);

                    for (const auto& neighbour : _maze.GetNeighbours(current.m_Coord)) {

                        if (const auto [nActive, nValue] = neighbour; nActive) {

                            const auto n = Utils::To1D(nValue, _maze.Size());

                            // Check if node is not already visited:
                            if (!closedSet.Contains(n) && !dupes.Contains(n)) {

                                // Add to dupe list:
                                while (dupes.Capacity() <= n) {
                                    dupes.Reserve(std::min(dupes.Capacity() * 2U, Utils::Product<size_t>(_maze.Size())));
                                }
                                dupes.Add(n);

                                // Create a parent node and transfer ownership of 'current' to it. Note: 'current' is now moved!
                                openSet.Emplace({ n, current.m_GScore + static_cast<Ts>(1), _h(nValue, _end), std::make_shared<ASNode>(std::move(current)) });
                            }
                        }
                    }
                }
                else { // SOLUTION REACHED ...

                    // Free data which is no longer relevant:
                      openSet.Clear();
                    closedSet.Clear();
                        dupes.Clear();

                    // Recurse from end node to start node, inserting into a result buffer:
                    result.reserve(current.m_GScore);
                    result.emplace_back(Utils::ToND(current.m_Coord, _maze.Size()));

                    if (current.m_Parent != nullptr) {

                        for (auto item = current.m_Parent; item->m_Parent != nullptr;) {
                            result.emplace_back(Utils::ToND(item->m_Coord, _maze.Size()));

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

            return result;
        }
    };

} // CHDR::Solvers

#endif //CHDR_ASTAR_HPP