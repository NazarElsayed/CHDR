/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_DFS_HPP
#define CHDR_DFS_HPP

#include <stack>

#include "base/ISolver.hpp"
#include "mazes/base/IMaze.hpp"
#include "types/ExistenceSet.hpp"
#include "utils/Utils.hpp"
#include "mazes/Grid.hpp"

namespace CHDR::Solvers {

    template<typename Tm, const size_t Kd>
    class DFS final {

        struct DFSNode final {

            size_t m_Coord;

            const DFSNode* m_Parent;

            [[nodiscard]] constexpr DFSNode() :
                m_Coord(-1U),
                m_Parent(nullptr) {}

            [[nodiscard]] constexpr DFSNode(const size_t &_coord, const DFSNode* const _parent) :
                m_Coord(_coord),
                m_Parent(std::move(_parent)) {}
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

                auto sequence = std::vector<DFSNode>(_capacity);
                std::stack<DFSNode, std::vector<DFSNode>> openSet(std::move(sequence));
                openSet.emplace(s, nullptr);

                ExistenceSet<LowMemoryUsage> closedSet({ s }, _capacity);

                std::vector<DFSNode*> buffer;

                while (!openSet.empty()) { // SEARCH FOR SOLUTION...

                    for (size_t i = 0U; i < openSet.size(); ++i) {

                        const DFSNode current(std::move(openSet.top()));
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

                                        if (closedSet.Capacity() > current.m_Coord) {
                                            closedSet.Reserve(std::min(_capacity * ((current.m_Coord % _capacity) + 1U), Utils::Product<size_t>(_maze.Size())));
                                        }
                                        closedSet.Add(current.m_Coord);

                                        // Create a parent node and transfer ownership of 'current' to it. Note: 'current' is now moved!
                                        buffer.emplace_back(new DFSNode(std::move(current)));
                                        openSet.Emplace({ n, buffer.back() });
                                    }
                                }
                            }
                        }
                        else { // SOLUTION REACHED ...

                            // Free data which is no longer relevant:
                            std::stack<DFSNode, std::vector<DFSNode>> empty;
                            std::swap(openSet, empty);

                            closedSet.Clear();

                            // Recurse from end node to start node, inserting into a result buffer:
                            result.reserve(_capacity);
                            for (const auto* temp = &current; temp->m_Parent != nullptr; temp = temp->m_Parent) {
                                result.emplace_back(Utils::ToND(temp->m_Coord, _maze.Size()));
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
                }
            }

            return result;
        }

    };

} // CHDR::Solvers

#endif //CHDR_DFS_HPP