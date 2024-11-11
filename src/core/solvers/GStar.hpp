/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_GSTAR_HPP
#define CHDR_GSTAR_HPP

#include <memory>

#include "base/BSolver.hpp"
#include "mazes/base/IMaze.hpp"
#include "mazes/Graph.hpp"
#include "mazes/Grid.hpp"
#include "types/ExistenceSet.hpp"
#include "types/Heap.hpp"
#include "utils/Utils.hpp"

namespace CHDR::Solvers {

    template<typename weight_t, const size_t Kd, typename scalar_t, typename index_t>
    class [[maybe_unused]] GStar final : public BSolver<weight_t, Kd, scalar_t, index_t> {

        static_assert(std::is_integral_v<scalar_t> || std::is_floating_point_v<scalar_t>, "scalar_t must be either an integral or floating point type");
        static_assert(std::is_integral_v<index_t>, "index_t must be an integral type.");

    private:

        using coord_t = Coord<index_t, Kd>;

        struct GSNode final : public ManagedNode<index_t> {

            scalar_t m_GScore;
            scalar_t m_FScore;

            [[nodiscard]] constexpr GSNode() : ManagedNode<index_t>() {};

            [[nodiscard]] constexpr GSNode(const index_t& _index, const scalar_t& _gScore, const scalar_t& _hScore) : ManagedNode<index_t>(_index),
                m_GScore(_gScore),
                m_FScore(_gScore + _hScore) {}

            [[nodiscard]] constexpr GSNode(const index_t& _index, const scalar_t& _gScore, const scalar_t& _hScore, GSNode&& _parent) : ManagedNode<index_t>(_index, std::move(_parent)),
                m_GScore(_gScore),
                m_FScore(_gScore + _hScore) {}

            struct Max {

                [[nodiscard]] constexpr bool operator () (const GSNode& _a, const GSNode& _b) const {

                    return _a.m_FScore == _b.m_FScore ?
                        _a.m_GScore > _b.m_GScore :
                        _a.m_FScore > _b.m_FScore;
                }
            };

            struct Min {

                [[nodiscard]] constexpr bool operator () (const GSNode& _a, const GSNode& _b) const {

                    return _a.m_FScore == _b.m_FScore ?
                        _a.m_GScore < _b.m_GScore :
                        _a.m_FScore < _b.m_FScore;
                }
            };
        };

    public:

        [[maybe_unused]]
        std::vector<coord_t> Execute(const Mazes::Graph<index_t, scalar_t>& _maze, const coord_t& _start, const coord_t& _end, const coord_t& _size, scalar_t (*_h)(const coord_t&, const coord_t&), const scalar_t& _weight, size_t _capacity) const override {

            std::vector<coord_t> result;

            const auto s = Utils::To1D(_start, _size);
            const auto e = Utils::To1D(_end,   _size);

            const auto count = _maze.Count();

            // Create closed set:
            _capacity = std::max(_capacity, std::max(s, e));
            ExistenceSet closed({ s }, _capacity);

            // Create open set:
            Heap<GSNode, 2U, typename GSNode::Max> open;
            open.Emplace(GSNode { s, static_cast<scalar_t>(0), _h(_start, _end) });

            // Main loop:
            while (!open.Empty()) {

                auto curr = open.PopTop();

                if (curr.m_Index != e) { // SEARCH FOR SOLUTION...

                    if (closed.Capacity() < curr.m_Index) {
                        closed.Reserve(std::min(_capacity * ((curr.m_Index % _capacity) + 1U), count));
                    }
                    closed.Add(curr.m_Index);

                    for (const auto& neighbour : _maze.GetNeighbours(curr.m_Index)) {

                        if (const auto& [nActive, nCoord] = neighbour; nActive) {

                            const auto& [n, nDistance] = neighbour;

                            // Check if node is not already visited:
                            if (!closed.Contains(n)) {

                                if (closed.Capacity() < n) {
                                    closed.Reserve(std::min(_capacity * ((n % _capacity) + 1U), count));
                                }
                                closed.Add(n);

                                // Create a parent node and transfer ownership of 'current' to it. Note: 'current' is now moved!
                                open.Emplace(GSNode { n, curr.m_GScore + static_cast<scalar_t>(1), _h(Utils::ToND(n, _size), _end) * _weight, std::move(curr) });
                            }
                        }
                    }
                }
                else { // SOLUTION REACHED ...

                    // Free data which is no longer relevant:
                      open.Clear();   open.Trim();
                    closed.Clear(); closed.Trim();

                    // Reserve space in result:
                    result.reserve(curr.m_GScore);

                    // Recurse from end node to start node, inserting into a result buffer:
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
            ExistenceSet closed({ s }, _capacity);

            // Create open set:
            Heap<GSNode, 2U, typename GSNode::Max> open;
            open.Emplace(GSNode { s, static_cast<scalar_t>(0), _h(_start, _end) });

            // Main loop:
            while (!open.Empty()) {

                auto curr = open.PopTop();

                if (curr.m_Index != e) { // SEARCH FOR SOLUTION...

                    if (closed.Capacity() < curr.m_Index) {
                        closed.Reserve(std::min(_capacity * ((curr.m_Index % _capacity) + 1U), count));
                    }
                    closed.Add(curr.m_Index);

                    for (const auto& neighbour : _maze.GetNeighbours(curr.m_Index)) {

                        if (const auto& [nActive, nCoord] = neighbour; nActive) {

                            const auto n = Utils::To1D(nCoord, _maze.Size());

                            // Check if node is not already visited:
                            if (!closed.Contains(n)) {

                                if (closed.Capacity() < n) {
                                    closed.Reserve(std::min(_capacity * ((n % _capacity) + 1U), count));
                                }
                                closed.Add(n);

                                // Create a parent node and transfer ownership of 'current' to it. Note: 'current' is now moved!
                                open.Emplace(GSNode { n, curr.m_GScore + static_cast<scalar_t>(1), _h(nCoord, _end) * _weight, std::move(curr) });
                            }
                        }
                    }
                }
                else { // SOLUTION REACHED ...

                    // Free data which is no longer relevant:
                      open.Clear();   open.Trim();
                    closed.Clear(); closed.Trim();

                    // Recurse from end node to start node, inserting into a result buffer:
                    result.reserve(curr.m_GScore);
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

            return result;
        }

    };

} // CHDR::Solvers

#endif //CHDR_GSTAR_HPP