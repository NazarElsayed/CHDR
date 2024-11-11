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

#include "base/BSolver.hpp"
#include "mazes/Graph.hpp"
#include "mazes/Grid.hpp"
#include "types/ExistenceSet.hpp"
#include "types/StableForwardBuf.hpp"
#include "utils/Utils.hpp"

namespace CHDR::Solvers {

    template<typename weight_t, const size_t Kd, typename scalar_t, typename index_t>
    class [[maybe_unused]] DFS final : public BSolver<weight_t, Kd, scalar_t, index_t> {

        static_assert(std::is_integral_v<index_t>, "index_t must be an integral type.");

    private:

        using coord_t = Coord<index_t, Kd>;

        struct DFSNode final : public UnmanagedNode<index_t> {

            /**
             * @brief Constructs an uninitialized DFSNode.
             *
             * This constructor creates an DFSNode with uninitialized members.
             */
            DFSNode() : UnmanagedNode<index_t>() {}

            [[nodiscard]] constexpr DFSNode(const index_t& _index, const UnmanagedNode<index_t>* RESTRICT const _parent) : UnmanagedNode<index_t>(_index, _parent) {}
        };

    public:

        [[maybe_unused]]
        std::vector<coord_t> Execute(const Mazes::Graph<index_t, scalar_t>& _maze, const coord_t& _start, const coord_t& _end, const coord_t& _size, scalar_t (*_h)(const coord_t&, const coord_t&), const scalar_t& _weight, size_t _capacity) const override {

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

                    // Create closed Set:
                    _capacity = std::max(_capacity, std::max(s, e));
                    ExistenceSet<LowMemoryUsage> closed({ s }, _capacity);

                    // Create open Set:
                    auto sequence = std::vector<DFSNode>(_capacity);
                    std::stack<DFSNode, std::vector<DFSNode>> open(std::move(sequence));
                    open.emplace(s, nullptr);

                    // Create buffer:
                    StableForwardBuf<DFSNode> buf;

                    // Main loop:
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

                                        const auto& n = Utils::To1D(nCoord, _size);

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

                                // Reserve space in result:
                                result.reserve(_capacity);

                                // Recurse from end node to start node, inserting into a result buffer:
                                for (const auto* temp = &curr; temp->m_Parent != nullptr; temp = static_cast<const DFSNode*>(temp->m_Parent)) {
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
        std::vector<coord_t> Execute(const Mazes::Grid<Kd, weight_t>& _maze, const coord_t& _start, const coord_t& _end, scalar_t (*_h)(const coord_t&, const coord_t&), const scalar_t& _weight, size_t _capacity) const override {

            std::vector<coord_t> result;

            const auto s = Utils::To1D(_start, _maze.Size());
            const auto e = Utils::To1D(_end,   _maze.Size());

            const auto count = _maze.Count();

            // Create closed set:
            _capacity = std::max(_capacity, std::max(s, e));
            ExistenceSet<LowMemoryUsage> closed({ s }, _capacity);

            // Create open set:
            auto sequence = std::vector<DFSNode>(_capacity);
            std::stack<DFSNode, std::vector<DFSNode>> open(std::move(sequence));
            open.emplace(s, nullptr);

            // Create buffer:
            StableForwardBuf<DFSNode> buf;

            // Main loop:
            while (!open.empty()) { // SEARCH FOR SOLUTION...

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

                    // Reserve space in result:
                    result.reserve(_capacity);

                    // Recurse from end node to start node, inserting into a result buffer:
                    for (const auto* temp = &curr; temp->m_Parent != nullptr; temp = static_cast<const DFSNode*>(temp->m_Parent)) {
                        result.emplace_back(Utils::ToND(temp->m_Index, _maze.Size()));
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

#endif //CHDR_DFS_HPP