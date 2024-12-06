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

#include "base/bsolver.hpp"
#include "mazes/graph.hpp"
#include "mazes/Grid.hpp"
#include "solvers/base/UnmanagedNode.hpp"
#include "types/existence_set.hpp"
#include "types/StableForwardBuf.hpp"
#include "utils/Utils.hpp"

namespace chdr::solvers {

    template<typename weight_t, const size_t Kd, typename scalar_t, typename index_t>
    class [[maybe_unused]] bfs final : public bsolver<weight_t, Kd, scalar_t, index_t> {

        static_assert(std::is_integral_v<index_t>, "index_t must be an integral type.");

    private:

        using coord_t = coord_t<index_t, Kd>;

        struct bfs_node final : public UnmanagedNode<index_t> {

            /**
             * @brief Constructs an uninitialized BFSNode.
             *
             * This constructor creates an BFSNode with uninitialized members.
             */
            bfs_node() : UnmanagedNode<index_t>() {}

            [[nodiscard]] constexpr bfs_node(const index_t& _index, const UnmanagedNode<index_t>* RESTRICT const _parent) : UnmanagedNode<index_t>(_index, _parent) {}
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
            existence_set<low_memory_usage> closed({s }, _capacity);

            // Create open:
            std::queue<bfs_node> open;
            open.emplace(s, nullptr);

            // Create buffer:
            StableForwardBuf<bfs_node> buf;

            // Main loop:
            while (!open.empty()) { // SEARCH FOR SOLUTION...

                auto curr(std::move(open.front()));
                open.pop();

                if (curr.m_index != e) {

                    if (closed.capacity() < curr.m_index) {
                        closed.reserve(std::min(_capacity * ((curr.m_index % _capacity) + 1U), count));
                    }
                    closed.add(curr.m_index);

                    for (const auto& neighbour : _maze.GetNeighbours(curr.m_index)) {

                        const auto& [n, nDistance] = neighbour;

                        // Check if node is not already visited:
                        if (!closed.contains(n)) {

                            if (closed.capacity() < n) {
                                closed.reserve(std::min(_capacity * ((n % _capacity) + 1U), count));
                            }
                            closed.add(n);

                            // Create a parent node and transfer ownership of 'current' to it. Note: 'current' is now moved!
                            open.push({ n, &buf.Emplace(std::move(curr)) });
                        }
                    }
                }
                else { // SOLUTION REACHED ...

                    curr.template Backtrack<bfs_node>(result, _size, _capacity);

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

            // Create closed:
            _capacity = std::max(_capacity, std::max(s, e));
            existence_set<low_memory_usage> closed({s }, _capacity);

            // Create open set:
            std::queue<bfs_node> open;
            open.emplace(s, nullptr);

            // Create buffer:
            StableForwardBuf<bfs_node> buf;

            // Main loop:
            while (!open.empty()) { // SEARCH FOR SOLUTION...

                for (size_t i = 0U; i < open.size(); ++i) {

                    auto curr(std::move(open.front()));
                    open.pop();

                    if (curr.m_index != e) {

                        if (closed.capacity() < curr.m_index) {
                            closed.reserve(std::min(_capacity * ((curr.m_index % _capacity) + 1U), count));
                        }
                        closed.add(curr.m_index);

                        for (const auto& neighbour: _maze.GetNeighbours(curr.m_index)) {

                            if (const auto& [nActive, nCoord] = neighbour; nActive) {

                                const auto n = Utils::To1D(nCoord, _maze.Size());

                                // Check if node is not already visited:
                                if (!closed.contains(n)) {

                                    if (closed.capacity() < n) {
                                        closed.reserve(std::min(_capacity * ((n % _capacity) + 1U), count));
                                    }
                                    closed.add(n);

                                    // Create a parent node and transfer ownership of 'current' to it. Note: 'current' is now moved!
                                    open.push({ n, &buf.Emplace(std::move(curr)) });
                                }
                            }
                        }
                    }
                    else { // SOLUTION REACHED ...

                        curr.template Backtrack<bfs_node>(result, _maze.Size(), _capacity);

                        break;
                    }
                }
            }

            return result;
        }
    };

} // CHDR::Solvers

#endif //CHDR_BFS_HPP