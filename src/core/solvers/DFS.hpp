/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_DFS_HPP
#define CHDR_DFS_HPP

#include <cmath>
#include <functional>
#include <stack>
#include <unordered_set>

#include "../utils/Heuristics.hpp"
#include "base/ISolver.hpp"
#include "mazes/base/IMaze.hpp"
#include "types/DenseExistenceSet.hpp"
#include "types/Heap.hpp"
#include "utils/Utils.hpp"
#include "mazes/Grid.hpp"

namespace CHDR::Solvers {

    template<typename Tm, const size_t Kd>
    class DFS final : public ISolver<Tm> {

        struct DFSNode_Managed final {

            size_t m_Coord;

            std::shared_ptr<const DFSNode_Managed> m_Parent;

            [[nodiscard]] constexpr DFSNode_Managed() :
                m_Coord(-1U),
                m_Parent(nullptr) {}

            [[nodiscard]] constexpr DFSNode_Managed(const size_t &_coord, const std::shared_ptr<const DFSNode_Managed>& _parent) :
                m_Coord(_coord),
                m_Parent(std::move(_parent)) {}

            ~DFSNode_Managed() {

                while (m_Parent && static_cast<unsigned>(m_Parent.use_count()) < 2U) {
                    m_Parent = std::move(m_Parent->m_Parent);
                }
            }
        };

        struct DFSNode_Unmanaged final {

            size_t m_Coord;

            const DFSNode_Unmanaged* m_Parent;

            [[nodiscard]] constexpr DFSNode_Unmanaged() :
                m_Coord(-1U),
                m_Parent(nullptr) {}

            [[nodiscard]] constexpr DFSNode_Unmanaged(const size_t &_coord, const DFSNode_Unmanaged* const _parent) :
                m_Coord(_coord),
                m_Parent(std::move(_parent)) {}
        };

    private:

        using coord_t = Coord<size_t, Kd>;

    public:

        void Solve(const Mazes::IMaze<Tm>& _maze) override {

            (void)_maze; // Suppress unused variable warning.

            throw std::runtime_error("AStar::Solve(const Mazes::IMaze& _maze) Not implemented!");
        }

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

                auto sequence = std::vector<DFSNode_Managed>(_capacity);
                std::stack<DFSNode_Managed, std::vector<DFSNode_Managed>> openSet(std::move(sequence));
                openSet.emplace(s, nullptr);

                DenseExistenceSet closedSet ({ s }, _capacity);
                DenseExistenceSet dupes     (       _capacity);

                std::vector<DFSNode_Unmanaged*> buffer;

                while (!openSet.empty()) { // SEARCH FOR SOLUTION...

                    for (size_t i = 0U; i < openSet.size(); ++i) {

                        const DFSNode_Managed current(std::move(openSet.top()));
                        openSet.pop();

                        if (current.m_Coord != e) {

                            if (closedSet.Capacity() > current.m_Coord) {
                                closedSet.Reserve(std::min(_capacity * ((current.m_Coord % _capacity) + 1U), Utils::Product<size_t>(_maze.Size())));
                            }
                            closedSet.Add(current.m_Coord);

                            for (const auto& neighbour: _maze.GetNeighbours(current.m_Coord)) {

                                if (const auto [nActive, nValue] = neighbour; nActive) {

                                    const auto n = Utils::To1D(nValue, _maze.Size());

                                    // Check if node is not already visited:
                                    if (!closedSet.Contains(n) && !dupes.Contains(n)) {

                                        // Add to dupe list:
                                        if (dupes.Capacity() > n) {
                                            dupes.Reserve(std::min(_capacity * ((n % _capacity) + 1U), Utils::Product<size_t>(_maze.Size())));
                                        }
                                        dupes.Add(n);

                                        // Create a parent node and transfer ownership of 'current' to it. Note: 'current' is now moved!
                                        openSet.emplace(n, std::make_shared<DFSNode_Managed>(std::move(current)));
                                    }
                                }
                            }
                        }
                        else { // SOLUTION REACHED ...

                            // Free data which is no longer relevant:
                            std::stack<DFSNode_Managed, std::vector<DFSNode_Managed>> empty;
                            std::swap(openSet, empty);

                            closedSet.Clear();
                                dupes.Clear();

                            // Recurse from end node to start node, inserting into a result buffer:
                            result.reserve(_capacity);
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
                }
            }

            return result;
        }

        auto SolveFaster(const Mazes::Grid<Kd, Tm>& _maze, const coord_t& _start, const coord_t& _end, size_t _capacity = 0U) {

            std::vector<coord_t> result;

            const auto s = Utils::To1D(_start, _maze.Size());
            const auto e = Utils::To1D(_end,   _maze.Size());

            if (_maze.Contains(s) &&
                _maze.Contains(e) &&
                _maze.At(s).IsActive() &&
                _maze.At(e).IsActive()
            ) {

                _capacity = std::max(_capacity, std::max(s, e));

                auto sequence = std::vector<DFSNode_Unmanaged>(_capacity);
                std::stack<DFSNode_Unmanaged, std::vector<DFSNode_Unmanaged>> openSet(std::move(sequence));
                openSet.emplace(s, nullptr);

                DenseExistenceSet closedSet ({ s }, _capacity);
                DenseExistenceSet dupes     (       _capacity);

                std::vector<DFSNode_Unmanaged*> buffer;

                while (!openSet.empty()) { // SEARCH FOR SOLUTION...

                    for (size_t i = 0U; i < openSet.size(); ++i) {

                        const DFSNode_Unmanaged current(std::move(openSet.top()));
                        openSet.pop();

                        if (current.m_Coord != e) {

                            if (closedSet.Capacity() > current.m_Coord) {
                                closedSet.Reserve(std::min(_capacity * ((current.m_Coord % _capacity) + 1U), Utils::Product<size_t>(_maze.Size())));
                            }
                            closedSet.Add(current.m_Coord);

                            for (const auto& neighbour: _maze.GetNeighbours(current.m_Coord)) {

                                if (const auto [nActive, nValue] = neighbour; nActive) {

                                    const auto n = Utils::To1D(nValue, _maze.Size());

                                    // Check if node is not already visited:
                                    if (!closedSet.Contains(n) && !dupes.Contains(n)) {

                                        // Add to dupe list:
                                        if (dupes.Capacity() > n) {
                                            dupes.Reserve(std::min(_capacity * ((n % _capacity) + 1U), Utils::Product<size_t>(_maze.Size())));
                                        }
                                        dupes.Add(n);

                                        // Create a parent node and transfer ownership of 'current' to it. Note: 'current' is now moved!
                                        buffer.emplace_back(new DFSNode_Unmanaged(std::move(current)));
                                        openSet.emplace(n, buffer.back());
                                    }
                                }
                            }
                        }
                        else { // SOLUTION REACHED ...

                            // Free data which is no longer relevant:
                            std::stack<DFSNode_Unmanaged, std::vector<DFSNode_Unmanaged>> empty;
                            std::swap(openSet, empty);

                            closedSet.Clear();
                                dupes.Clear();

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

#endif //CHDR_FLOODFILL_HPP