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

#include "base/BSolver.hpp"
#include "mazes/base/IMaze.hpp"
#include "mazes/Graph.hpp"
#include "mazes/Grid.hpp"
#include "types/ExistenceSet.hpp"
#include "types/StableForwardBuf.hpp"
#include "utils/Utils.hpp"

namespace CHDR::Solvers {

    template<typename weight_t, const size_t Kd, typename scalar_t, typename index_t>
    class [[maybe_unused]] GBFS final : BSolver<weight_t, Kd, scalar_t, index_t> {

        static_assert(std::is_integral_v<index_t>, "index_t must be an integral type.");

    private:

        using coord_t = Coord<index_t, Kd>;

        struct GBFSNode final : public ManagedNode<index_t> {

            /**
             * @brief Constructs an uninitialized GBFSNode.
             *
             * This constructor creates an GBFSNode with uninitialized members.
             */
            [[nodiscard]] constexpr GBFSNode() : UnmanagedNode<index_t>() {} // NOLINT(*-pro-type-member-init, *-use-equals-default)

            [[nodiscard]] constexpr GBFSNode(const index_t& _index) {}

            [[nodiscard]] constexpr GBFSNode(const index_t& _index, GBFSNode&& _parent) :
                m_Index(_index)
            {
                m_Parent = std::make_shared<const GBFSNode>(std::move(_parent));
            }
        };


    public:

        [[maybe_unused]]
        auto Solve(const Mazes::Graph<index_t, scalar_t>& _maze, const coord_t& _start, const coord_t& _end, const coord_t& _size, size_t _capacity = 0U) {

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

                    std::queue<GBFSNode> open;
                    open.emplace(s);

                    ExistenceSet closed({ s }, _capacity);

                    while (!open.empty()) { // SEARCH FOR SOLUTION...

                        for (size_t i = 0U; i < open.size(); ++i) {

                            auto curr(std::move(open.front()));
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
                                            open.emplace(n, std::move(curr));
                                        }
                                    }
                                }
                            }
                            else { // SOLUTION REACHED ...

                                // Free data which is no longer relevant:
                                std::queue<GBFSNode> empty;
                                std::swap(open, empty);

                                closed.Clear(); closed.Trim();

                                // Recurse from end node to start node, inserting into a result buffer:
                                result.reserve(_capacity);
                                result.emplace_back(Utils::ToND(curr.m_Index, _size));

                                if (curr.m_Parent != nullptr) {

                                    for (auto& item = curr.m_Parent; item->m_Parent != nullptr;) {
                                        result.emplace_back(Utils::ToND(item->m_Index, _size));

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
                else {
                    result.emplace_back(_end);
                }
            }

            return result;
        }

        [[maybe_unused]]
        auto Solve(const Mazes::Grid<Kd, weight_t>& _maze, const coord_t& _start, const coord_t& _end, size_t _capacity = 0U) {

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

                    std::queue<GBFSNode> open;
                    open.emplace(s, nullptr);

                    ExistenceSet closed({ s }, _capacity);

                    while (!open.empty()) { // SEARCH FOR SOLUTION...

                        for (size_t i = 0U; i < open.size(); ++i) {

                            auto curr(std::move(open.front()));
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
                                            open.emplace(n, std::move(curr));
                                        }
                                    }
                                }
                            }
                            else { // SOLUTION REACHED ...

                                // Free data which is no longer relevant:
                                std::queue<GBFSNode> empty;
                                std::swap(open, empty);

                                closed.Clear(); closed.Trim();

                                // Recurse from end node to start node, inserting into a result buffer:
                                result.reserve(_capacity);
                                result.emplace_back(Utils::ToND(curr.m_Index, _maze.Size()));

                                if (curr.m_Parent != nullptr) {

                                    for (auto& item = curr.m_Parent; item->m_Parent != nullptr;) {
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
                }
                else {
                    result.emplace_back(_end);
                }
            }

            return result;
        }

    };

} // CHDR::Solvers

#endif //CHDR_GBFS_HPP