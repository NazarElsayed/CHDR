/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_BFS_HPP
#define CHDR_BFS_HPP

#include "base/bsolver.hpp"
#include "mazes/graph.hpp"
#include "mazes/grid.hpp"
#include "solvers/base/unmanaged_node.hpp"
#include "types/existence_set.hpp"
#include "types/queue.hpp"
#include "types/stable_forward_buf.hpp"
#include "utils/utils.hpp"

namespace chdr::solvers {

    template<typename weight_t, const size_t Kd, typename scalar_t, typename index_t>
    class [[maybe_unused]] bfs final : public bsolver<weight_t, Kd, scalar_t, index_t> {

        static_assert(std::is_integral_v<index_t>, "index_t must be an integral type.");

    private:

        using coord_t = coord<index_t, Kd>;

        struct bfs_node final : unmanaged_node<index_t> {

            /**
             * @brief Constructs an uninitialized BFSNode.
             *
             * This constructor creates an BFSNode with uninitialized members.
             */
            bfs_node() : unmanaged_node<index_t>() {}

            [[nodiscard]] constexpr bfs_node(const index_t& _index, const unmanaged_node<index_t>* RESTRICT const _parent) : unmanaged_node<index_t>(_index, _parent) {}
        };

    public:

        [[maybe_unused]]
        std::vector<coord_t> execute(const mazes::graph<index_t, scalar_t>& _maze, const coord_t& _start, const coord_t& _end, const coord_t& _size, scalar_t (*_h)(const coord_t&, const coord_t&), const scalar_t& _weight, size_t _capacity) const override {

            std::vector<coord_t> result;

            const auto s = utils::to_1d(_start, _size);
            const auto e = utils::to_1d(_end,   _size);

            const auto count = _maze.count();

            // Create closed set:
            _capacity = std::max(_capacity, std::max(s, e));
            existence_set<low_memory_usage> closed({ s }, _capacity);

            // Create open:
            queue<bfs_node> open;
            open.emplace(s, nullptr);

            // Create buffer:
            stable_forward_buf<bfs_node> buf;

            // Main loop:
            while (!open.empty()) { // SEARCH FOR SOLUTION...

                auto curr(std::move(open.top()));
                open.pop();

                if (curr.m_index != e) {

                    if (closed.capacity() < curr.m_index) {
                        closed.reserve(std::min(_capacity * ((curr.m_index % _capacity) + 1U), count));
                    }
                    closed.emplace(curr.m_index);

                    for (const auto& neighbour : _maze.get_neighbours(curr.m_index)) {

                        const auto& [n, nDistance] = neighbour;

                        // Check if node is not already visited:
                        if (!closed.contains(n)) {

                            if (closed.capacity() < n) {
                                closed.reserve(std::min(_capacity * ((n % _capacity) + 1U), count));
                            }
                            closed.emplace(n);

                            // Create a parent node and transfer ownership of 'current' to it. Note: 'current' is now moved!
                            open.emplace(n, &buf.emplace(std::move(curr)));
                        }
                    }
                }
                else { // SOLUTION REACHED ...

                    curr.template backtrack<bfs_node>(result, _size, _capacity);

                    break;
                }
            }

            return result;
        }

        [[maybe_unused]]
        std::vector<coord_t> execute(const mazes::grid<Kd, weight_t>& _maze, const coord_t& _start, const coord_t& _end, scalar_t (*_h)(const coord_t&, const coord_t&), const scalar_t& _weight, size_t _capacity) const override {

            std::vector<coord_t> result;

            const auto s = utils::to_1d(_start, _maze.size());
            const auto e = utils::to_1d(_end, _maze.size());

            const auto count = _maze.count();

            // Create closed:
            _capacity = std::max(_capacity, std::max(s, e));
            existence_set<low_memory_usage> closed({ s }, _capacity);

            // Create open set:
            queue<bfs_node> open;
            open.emplace(s, nullptr);

            // Create buffer:
            stable_forward_buf<bfs_node> buf;

            // Main loop:
            while (!open.empty()) { // SEARCH FOR SOLUTION...

                for (size_t i = 0U; i < open.size(); ++i) {

                    auto curr(std::move(open.top()));
                    open.pop();

                    if (curr.m_index != e) {

                        if (closed.capacity() < curr.m_index) {
                            closed.reserve(std::min(_capacity * ((curr.m_index % _capacity) + 1U), count));
                        }
                        closed.emplace(curr.m_index);

                        for (const auto& neighbour: _maze.get_neighbours(curr.m_index)) {

                            if (const auto& [nActive, nCoord] = neighbour; nActive) {

                                const auto n = utils::to_1d(nCoord, _maze.size());

                                // Check if node is not already visited:
                                if (!closed.contains(n)) {

                                    if (closed.capacity() < n) {
                                        closed.reserve(std::min(_capacity * ((n % _capacity) + 1U), count));
                                    }
                                    closed.emplace(n);

                                    // Create a parent node and transfer ownership of 'current' to it. Note: 'current' is now moved!
                                    open.emplace(n, &buf.emplace(std::move(curr)));
                                }
                            }
                        }
                    }
                    else { // SOLUTION REACHED ...

                        curr.template backtrack<bfs_node>(result, _maze.size(), _capacity);

                        break;
                    }
                }
            }

            return result;
        }
    };

} //chdr::solvers

#endif //CHDR_BFS_HPP