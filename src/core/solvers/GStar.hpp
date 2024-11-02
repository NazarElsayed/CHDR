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
#include "types/StableForwardBuf.hpp"
#include "types/StackAllocator.hpp"
#include "utils/Utils.hpp"

namespace CHDR::Solvers {

    template<typename weight_t, const size_t Kd, typename scalar_t, typename index_t>
    class [[maybe_unused]] GStar final : BSolver<weight_t, Kd, scalar_t, index_t> {

        static_assert(std::is_integral_v<scalar_t> || std::is_floating_point_v<scalar_t>, "scalar_t must be either an integral or floating point type");
        static_assert(std::is_integral_v<index_t>, "index_t must be an integral type.");

    private:

        using coord_t = Coord<index_t, Kd>;

        struct GSNode final {

            index_t m_Index;

            scalar_t m_GScore;
            scalar_t m_FScore;

            std::shared_ptr<const GSNode> m_Parent;

            /**
             * @brief Constructs an uninitialized ASNode.
             *
             * This constructor creates an ASNode with uninitialized members.
             */
            [[nodiscard]] constexpr GSNode() {} // NOLINT(*-pro-type-member-init, *-use-equals-default)

            [[nodiscard]] constexpr GSNode(const index_t& _coord, const scalar_t& _gScore, const scalar_t& _hScore) :
                m_Index(_coord),
                m_GScore(_gScore),
                m_FScore(_gScore + _hScore),
                m_Parent() {}

            [[nodiscard]] constexpr GSNode(const index_t& _coord, const scalar_t& _gScore, const scalar_t& _hScore, GSNode&& _parent) :
                m_Index(_coord),
                m_GScore(_gScore),
                m_FScore(_gScore + _hScore)
            {
                m_Parent = std::make_shared<const GSNode>(std::move(_parent));
            }

            ~GSNode() { // NOLINT(*-use-equals-default)

//                while (m_Parent && m_Parent.unique()) {
//                    m_Parent = std::move(m_Parent->m_Parent);
//                }

                Expunge_Recursive(m_Parent);
            }

            void Expunge_Recursive(std::shared_ptr<const GSNode>& _node) {
                if (_node && _node.unique()) {
                    _node = std::move(_node->m_Parent);
                    Expunge_Recursive(_node);
                }
            }

            [[nodiscard]] constexpr bool operator == (const GSNode& _node) const { return m_Index == _node.m_Index; }

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
        auto Solve(const Mazes::Graph<index_t, scalar_t>& _maze, const coord_t& _start, const coord_t& _end, const coord_t& _size, scalar_t (*_h)(const coord_t&, const coord_t&), const scalar_t& _weight = 1, size_t _capacity = 0U) const {

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

                    ExistenceSet closed({ s }, _capacity);

                    Heap<GSNode, 2U, typename GSNode::Max> open;
                    open.Emplace(GSNode { s, static_cast<scalar_t>(0), _h(_start, _end) });

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

                            // Recurse from end node to start node, inserting into a result buffer:
                            result.reserve(curr.m_GScore);
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
                else {
                    result.emplace_back(_end);
                }
            }

            return result;
        }

        [[maybe_unused]]
        auto Solve(const Mazes::Grid<Kd, weight_t>& _maze, const coord_t& _start, const coord_t& _end, scalar_t (*_h)(const coord_t&, const coord_t&), const scalar_t& _weight = 1, size_t _capacity = 0U) const {

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

                    ExistenceSet closed({ s }, _capacity);

                    Heap<GSNode, 2U, typename GSNode::Max> open;
                    open.Emplace(GSNode { s, static_cast<scalar_t>(0), _h(_start, _end) });

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
                }
                else {
                    result.emplace_back(_end);
                }
            }

            return result;
        }

    };

} // CHDR::Solvers

#endif //CHDR_GSTAR_HPP