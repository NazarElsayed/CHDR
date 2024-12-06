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

#include "base/bsolver.hpp"
#include "mazes/base/IMaze.hpp"
#include "mazes/graph.hpp"
#include "mazes/Grid.hpp"
#include "types/existence_set.hpp"
#include "types/heap.hpp"
#include "utils/Utils.hpp"

namespace chdr::solvers {

    template<typename weight_t, const size_t Kd, typename scalar_t, typename index_t>
    class [[maybe_unused]] GStar final : public bsolver<weight_t, Kd, scalar_t, index_t> {

        static_assert(std::is_integral_v<scalar_t> || std::is_floating_point_v<scalar_t>, "scalar_t must be either an integral or floating point type");
        static_assert(std::is_integral_v<index_t>, "index_t must be an integral type.");

    private:

        using coord_t = coord_t<index_t, Kd>;

        struct GSNode final : public ManagedNode<index_t> {

            scalar_t m_gScore;
            scalar_t m_fScore;

            [[nodiscard]] constexpr GSNode() : ManagedNode<index_t>() {};

            [[nodiscard]] constexpr GSNode(const index_t& _index, const scalar_t& _gScore, const scalar_t& _hScore) : ManagedNode<index_t>(_index),
                m_gScore(_gScore),
                m_fScore(_gScore + _hScore) {}

            [[nodiscard]] constexpr GSNode(const index_t& _index, const scalar_t& _gScore, const scalar_t& _hScore, GSNode&& _parent) : ManagedNode<index_t>(_index, std::move(_parent)),
                m_gScore(_gScore),
                m_fScore(_gScore + _hScore) {}

            struct Max {

                [[nodiscard]] constexpr bool operator () (const GSNode& _a, const GSNode& _b) const {

                    return _a.m_fScore == _b.m_fScore ?
                        _a.m_gScore > _b.m_gScore :
                        _a.m_fScore > _b.m_fScore;
                }
            };

            struct Min {

                [[nodiscard]] constexpr bool operator () (const GSNode& _a, const GSNode& _b) const {

                    return _a.m_fScore == _b.m_fScore ?
                        _a.m_gScore < _b.m_gScore :
                        _a.m_fScore < _b.m_fScore;
                }
            };
        };

    public:

        [[maybe_unused]]
        std::vector<coord_t> execute(const mazes::graph<index_t, scalar_t>& _maze, const coord_t& _start, const coord_t& _end, const coord_t& _size, scalar_t (*_h)(const coord_t&, const coord_t&), const scalar_t& _weight, size_t _capacity) const override {

            std::vector<coord_t> result;

            const auto s = Utils::To1D(_start, _size);
            const auto e = Utils::To1D(_end,   _size);

            const auto count = _maze.Count();

            // Create closed set:
            _capacity = std::max(_capacity, std::max(s, e));
            existence_set closed({s }, _capacity);

            // Create open set:
            heap<GSNode, 2U, typename GSNode::Max> open;
            open.Emplace(GSNode { s, static_cast<scalar_t>(0), _h(_start, _end) });

            // Main loop:
            while (!open.Empty()) {

                auto curr = open.PopTop();

                if (curr.m_index != e) { // SEARCH FOR SOLUTION...

                    if (closed.capacity() < curr.m_index) {
                        closed.reserve(std::min(_capacity * ((curr.m_index % _capacity) + 1U), count));
                    }
                    closed.add(curr.m_index);

                    for (const auto& neighbour : _maze.GetNeighbours(curr.m_index)) {

                        if (const auto& [nActive, nCoord] = neighbour; nActive) {

                            const auto& [n, nDistance] = neighbour;

                            // Check if node is not already visited:
                            if (!closed.contains(n)) {

                                if (closed.capacity() < n) {
                                    closed.reserve(std::min(_capacity * ((n % _capacity) + 1U), count));
                                }
                                closed.add(n);

                                // Create a parent node and transfer ownership of 'current' to it. Note: 'current' is now moved!
                                open.Emplace(GSNode { n, curr.m_gScore + static_cast<scalar_t>(1), _h(Utils::ToND(n, _size), _end) * _weight, std::move(curr) });
                            }
                        }
                    }
                }
                else { // SOLUTION REACHED ...

                    // Free data which is no longer relevant:
                      open.Clear();   open.Trim();
                    closed.clear();
                    closed.trim();

                    curr.template Backtrack<GSNode>(result, _size, curr.m_gScore);

                    break;
                }
            }

            return result;
        }

        [[maybe_unused]]
        std::vector<coord_t> Execute(const mazes::Grid<Kd, weight_t>& _maze, const coord_t& _start, const coord_t& _end, scalar_t (*_h)(const coord_t&, const coord_t&), const scalar_t& _weight, size_t _capacity) const override {

            std::vector<coord_t> result;

            const auto s = Utils::To1D(_start, _maze.Size());
            const auto e = Utils::To1D(_end,   _maze.Size());

            const auto count = _maze.Count();

            // Create closed set:
            _capacity = std::max(_capacity, std::max(s, e));
            existence_set closed({s }, _capacity);

            // Create open set:
            heap<GSNode, 2U, typename GSNode::Max> open;
            open.Emplace(GSNode { s, static_cast<scalar_t>(0), _h(_start, _end) });

            // Main loop:
            while (!open.Empty()) {

                auto curr = open.PopTop();

                if (curr.m_index != e) { // SEARCH FOR SOLUTION...

                    if (closed.capacity() < curr.m_index) {
                        closed.reserve(std::min(_capacity * ((curr.m_index % _capacity) + 1U), count));
                    }
                    closed.add(curr.m_index);

                    for (const auto& neighbour : _maze.GetNeighbours(curr.m_index)) {

                        if (const auto& [nActive, nCoord] = neighbour; nActive) {

                            const auto n = Utils::To1D(nCoord, _maze.Size());

                            // Check if node is not already visited:
                            if (!closed.contains(n)) {

                                if (closed.capacity() < n) {
                                    closed.reserve(std::min(_capacity * ((n % _capacity) + 1U), count));
                                }
                                closed.add(n);

                                // Create a parent node and transfer ownership of 'current' to it. Note: 'current' is now moved!
                                open.Emplace(GSNode { n, curr.m_gScore + static_cast<scalar_t>(1), _h(nCoord, _end) * _weight, std::move(curr) });
                            }
                        }
                    }
                }
                else { // SOLUTION REACHED ...

                    // Free data which is no longer relevant:
                      open.Clear();   open.Trim();
                    closed.clear();
                    closed.trim();

                    curr.template Backtrack<GSNode>(result, _maze.Size(), curr.m_gScore);

                    break;
                }
            }

            return result;
        }

    };

} // CHDR::Solvers

#endif //CHDR_GSTAR_HPP