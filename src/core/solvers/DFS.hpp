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
#include "mazes/Graph.hpp"
#include "mazes/Grid.hpp"
#include "types/ExistenceSet.hpp"
#include "types/StableForwardBuf.hpp"
#include "utils/Utils.hpp"

namespace CHDR::Solvers {

    template<typename Tm, const size_t Kd, typename Ti>
    class [[maybe_unused]] DFS final {

        static_assert(std::is_integral_v<Ti>, "Ti must be an integral type.");

    private:

        using coord_t = Coord<Ti, Kd>;

        struct DFSNode final {

            Ti m_Index;

            const DFSNode* RESTRICT m_Parent;

            /**
             * @brief Constructs an uninitialized ASNode.
             *
             * This constructor creates an ASNode with uninitialized members.
             */
            [[nodiscard]] constexpr DFSNode() {} // NOLINT(*-pro-type-member-init, *-use-equals-default)

            [[nodiscard]] constexpr DFSNode(const Ti &_index, const DFSNode* RESTRICT const _parent) :
                m_Index(_index),
                m_Parent(std::move(_parent)) {}
        };

    public:

        template <typename Ts>
        [[maybe_unused]]
        auto Solve(const Mazes::Graph<Ti, Ts>& _maze, const coord_t& _start, const coord_t& _end, const coord_t& _size, size_t _capacity = 0U) {

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

                    auto sequence = std::vector<DFSNode>(_capacity);
                    std::stack<DFSNode, std::vector<DFSNode>> open(std::move(sequence));
                    open.emplace(s, nullptr);

                    ExistenceSet<LowMemoryUsage> closed({ s }, _capacity);

                    StableForwardBuf<DFSNode> buf;

                    while (!open.empty()) { // SEARCH FOR SOLUTION...

                        for (size_t i = 0U; i < open.size(); ++i) {

                            auto curr(std::move(open.top()));
                            open.pop();

                            if (curr.m_Index != e) {

                                if (closed.Capacity() < curr.m_Index) {
                                    closed.Reserve(std::min(_capacity * ((curr.m_Index % _capacity) + 1U), count));
                                }
                                closed.Add(curr.m_Index);

                                for (const auto& neighbour: _maze.GetNeighbours(curr.m_Index)) {

                                    if (const auto& [nActive, nCoord] = neighbour; nActive) {

                                        const auto& [n, nDistance] = neighbour;

                                        // Check if node is not already visited:
                                        if (!closed.Contains(n)) {

                                            if (closed.Capacity() < n) {
                                                closed.Reserve(std::min(_capacity * ((n % _capacity) + 1U), count));
                                            }
                                            closed.Add(n);

                                            // Create a parent node and transfer ownership of 'current' to it. Note: 'current' is now moved!
                                            open.push({n, &buf.Emplace(std::move(curr)) });
                                        }
                                    }
                                }
                            }
                            else { // SOLUTION REACHED ...

                                // Recurse from end node to start node, inserting into a result buffer:
                                result.reserve(_capacity);
                                for (const auto* temp = &curr; temp->m_Parent != nullptr; temp = temp->m_Parent) {
                                    result.emplace_back(Utils::ToND(temp->m_Index, _size));
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

        [[maybe_unused]]
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

                    auto sequence = std::vector<DFSNode>(_capacity);
                    std::stack<DFSNode, std::vector<DFSNode>> open(std::move(sequence));
                    open.emplace(s, nullptr);

                    ExistenceSet<LowMemoryUsage> closed({ s }, _capacity);

                    StableForwardBuf<DFSNode> buf;

                    while (!open.empty()) { // SEARCH FOR SOLUTION...

                        for (size_t i = 0U; i < open.size(); ++i) {

                            auto curr(std::move(open.top()));
                            open.pop();

                            if (curr.m_Index != e) {

                                if (closed.Capacity() < curr.m_Index) {
                                    closed.Reserve(std::min(_capacity * ((curr.m_Index % _capacity) + 1U), count));
                                }
                                closed.Add(curr.m_Index);

                                for (const auto& neighbour: _maze.GetNeighbours(curr.m_Index)) {

                                    if (const auto& [nActive, nCoord] = neighbour; nActive) {

                                        const auto n = Utils::To1D(nCoord, _maze.Size());

                                        // Check if node is not already visited:
                                        if (!closed.Contains(n)) {

                                            if (closed.Capacity() < n) {
                                                closed.Reserve(std::min(_capacity * ((n % _capacity) + 1U), count));
                                            }
                                            closed.Add(n);

                                            // Create a parent node and transfer ownership of 'current' to it. Note: 'current' is now moved!
                                            open.push({n, &buf.Emplace(std::move(curr)) });
                                        }
                                    }
                                }
                            }
                            else { // SOLUTION REACHED ...

                                // Recurse from end node to start node, inserting into a result buffer:
                                result.reserve(_capacity);
                                for (const auto* temp = &curr; temp->m_Parent != nullptr; temp = temp->m_Parent) {
                                    result.emplace_back(Utils::ToND(temp->m_Index, _maze.Size()));
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

#endif //CHDR_DFS_HPP