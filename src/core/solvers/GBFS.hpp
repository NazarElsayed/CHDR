/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_GBFS_HPP
#define CHDR_GBFS_HPP

#include <queue>

#include "base/ISolver.hpp"
#include "mazes/base/IMaze.hpp"
#include "types/ExistenceSet.hpp"
#include "utils/Utils.hpp"
#include "mazes/Grid.hpp"

namespace CHDR::Solvers {

    template<typename Tm, const size_t Kd>
    class GBFS final {

        struct GBFSNode final {

            size_t m_Coord;

            std::shared_ptr<const GBFSNode> m_Parent;

            [[nodiscard]] constexpr GBFSNode() :
                m_Coord(-1U),
                m_Parent(nullptr) {}

            [[nodiscard]] constexpr GBFSNode(const size_t &_coord, const std::shared_ptr<const GBFSNode>& _parent) :
                m_Coord(_coord),
                m_Parent(std::move(_parent)) {}

            ~GBFSNode() {

                while (m_Parent && static_cast<unsigned>(m_Parent.use_count()) < 2U) {
                    m_Parent = std::move(m_Parent->m_Parent);
                }
            }
        };

    private:

        using coord_t = Coord<size_t, Kd>;

    public:

        auto Solve(const Mazes::Grid<Kd, Tm>& _maze, const coord_t& _start, const coord_t& _end, size_t _capacity = 0U) {

            std::vector<coord_t> result;

            const auto s = Utils::To1D(_start, _maze.Size());
            const auto e = Utils::To1D(_end,   _maze.Size());

            if (_maze.Contains(s) &&
                _maze.Contains(e) &&
                _maze.At(s).IsActive() &&
                _maze.At(e).IsActive()
            ) {

                _capacity = std::max(_capacity, std::max(s, e));

                std::queue<GBFSNode, std::vector<GBFSNode>> openSet;
                openSet.emplace(s, nullptr);

                ExistenceSet closedSet({ s }, _capacity);

                while (!openSet.empty()) { // SEARCH FOR SOLUTION...

                    for (size_t i = 0U; i < openSet.size(); ++i) {

                        const GBFSNode current(std::move(openSet.front()));
                        openSet.pop();

                        if (current.m_Coord != e) {

                            if (closedSet.Capacity() > current.m_Coord) {
                                closedSet.Reserve(std::min(_capacity * ((current.m_Coord % _capacity) + 1U), Utils::Product<size_t>(_maze.Size())));
                            }
                            closedSet.Add(current.m_Coord);

                            for (const auto& neighbour: _maze.GetNeighbours(current.m_Coord)) {

                                if (const auto& [nActive, nCoord] = neighbour; nActive) {

                                    const auto n = Utils::To1D(nCoord, _maze.Size());

                                    // Check if node is not already visited:
                                    if (!closedSet.Contains(n)) {

                                        // Add to dupe list:
                                        if (closedSet.Capacity() > n) {
                                            closedSet.Reserve(std::min(_capacity * ((n % _capacity) + 1U), Utils::Product<size_t>(_maze.Size())));
                                        }
                                        closedSet.Add(n);

                                        // Create a parent node and transfer ownership of 'current' to it. Note: 'current' is now moved!
                                        openSet.emplace(n, std::make_shared<GBFSNode>(std::move(current)));
                                    }
                                }
                            }
                        }
                        else { // SOLUTION REACHED ...

                            // Free data which is no longer relevant:
                            std::queue<GBFSNode> empty;
                            std::swap(openSet, empty);

                            closedSet.Clear();

                            // Recurse from end node to start node, inserting into a result buffer:
                            result.reserve(_capacity);
                            result.emplace_back(Utils::ToND(current.m_Coord, _maze.Size()));

                            if (current.m_Parent != nullptr) {

                                for (auto& item = current.m_Parent; item->m_Parent != nullptr;) {
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
                }
            }

            return result;
        }

    };

} // CHDR::Solvers

#endif //CHDR_GBFS_HPP