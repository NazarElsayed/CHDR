/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_BFS_HPP
#define CHDR_BFS_HPP

#include <queue>

#include "base/ISolver.hpp"
#include "mazes/base/IMaze.hpp"
#include "types/ExistenceSet.hpp"
#include "utils/Utils.hpp"
#include "mazes/Grid.hpp"

namespace CHDR::Solvers {

    template<typename Tm, const size_t Kd>
    class BFS final {

        struct BFSNode final {

            size_t m_Coord;

            const BFSNode* m_Parent;

            /**
             * @brief Constructs an uninitialized ASNode.
             *
             * This constructor creates an ASNode with uninitialized members.
             */
            [[nodiscard]] constexpr BFSNode() {} // NOLINT(*-pro-type-member-init, *-use-equals-default)

            [[nodiscard]] constexpr BFSNode(const size_t& _coord, const BFSNode* const _parent) :
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

                if (s != e) {

                    const auto count = _maze.Count();

                    _capacity = std::max(_capacity, std::max(s, e));

                    std::queue<BFSNode> open;
                    open.emplace(s, nullptr);

                    ExistenceSet<LowMemoryUsage> closed({ s }, _capacity);

                    StableForwardBuf<BFSNode> buf;

                    while (!open.empty()) { // SEARCH FOR SOLUTION...

                        for (size_t i = 0U; i < open.size(); ++i) {

                            const auto curr(std::move(open.front()));
                            open.pop();

                            if (curr.m_Coord != e) {

                                if (closed.Capacity() > curr.m_Coord) {
                                    closed.Reserve(std::min(_capacity * ((curr.m_Coord % _capacity) + 1U), count));
                                }
                                closed.Add(curr.m_Coord);

                                for (const auto& neighbour: _maze.GetNeighbours(curr.m_Coord)) {

                                    if (const auto& [nActive, nCoord] = neighbour; nActive) {

                                        const auto n = Utils::To1D(nCoord, _maze.Size());

                                        // Check if node is not already visited:
                                        if (!closed.Contains(n)) {

                                            if (closed.Capacity() > curr.m_Coord) {
                                                closed.Reserve(std::min(_capacity * ((curr.m_Coord % _capacity) + 1U), count));
                                            }
                                            closed.Add(curr.m_Coord);

                                            // Create a parent node and transfer ownership of 'current' to it. Note: 'current' is now moved!
                                            open.push({ n, &buf.Emplace(std::move(curr)) });
                                        }
                                    }
                                }
                            }
                            else { // SOLUTION REACHED ...

                                // Recurse from end node to start node, inserting into a result buffer:
                                result.reserve(_capacity);
                                for (const auto* temp = &curr; temp->m_Parent != nullptr; temp = temp->m_Parent) {
                                    result.emplace_back(Utils::ToND(temp->m_Coord, _maze.Size()));
                                }

                                // Reverse the result:
                                std::reverse(result.begin(), result.end());

                                break;
                            }
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

#endif //CHDR_BFS_HPP