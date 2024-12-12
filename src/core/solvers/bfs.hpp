/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_BFS_HPP
#define CHDR_BFS_HPP

#include "mazes/graph.hpp"
#include "mazes/grid.hpp"
#include "solvers/base/unmanaged_node.hpp"
#include "types/existence_set.hpp"
#include "types/queue.hpp"
#include "types/stable_forward_buf.hpp"
#include "utils/utils.hpp"

namespace chdr::solvers {

    template<size_t Kd, typename scalar_t, typename index_t, typename params_t>
    struct [[maybe_unused]] bfs final {

        friend struct solver<bfs, Kd, scalar_t, index_t, params_t>;

        static_assert(std::is_integral_v<index_t>, "index_t must be an integral type.");

    private:

        using    coord_t = coord<index_t, Kd>;
        using bfs_node_t = unmanaged_node<index_t>;

        [[maybe_unused, nodiscard]] static constexpr std::vector<coord_t> execute(const params_t& _params) {

            std::vector<coord_t> result;

            const auto s = utils::to_1d(_params._start, _params._maze.size());
            const auto e = utils::to_1d(_params._end,   _params._maze.size());

            // Create closed set:
            const auto capacity = std::max(_params._capacity, std::max(s, e));
            existence_set<low_memory_usage> closed({ s }, capacity);

            // Create open:
            queue<bfs_node_t> open;
            open.emplace(s, nullptr);

            // Create buffer:
            stable_forward_buf<bfs_node_t> buf;

            // Main loop:
            while (!open.empty()) { // SEARCH FOR SOLUTION...

                auto curr(std::move(open.top()));
                open.pop();

                if (curr.m_index != e) {

                    closed.allocate(curr.m_index, capacity, _params._maze.count());
                    closed.emplace(curr.m_index);

                    for (const auto& neighbour : _params._maze.get_neighbours(curr.m_index)) {

                        if constexpr (std::is_same_v<std::decay_t<decltype(_params._maze)>, mazes::graph<index_t, scalar_t>>) {

                            const auto& [n, nDistance] = neighbour;

                            // Check if node is not already visited:
                            if (!closed.contains(n)) {
                                 closed.allocate(n, capacity, _params._maze.count());
                                 closed.emplace(n);

                                // Create a parent node and transfer ownership of 'current' to it. Note: 'current' is now moved!
                                open.emplace(n, &buf.emplace(std::move(curr)));
                            }
                        }
                        else {

                            if (const auto& [nActive, nCoord] = neighbour; nActive) {

                                const auto n = utils::to_1d(nCoord, _params._maze.size());

                                // Check if node is not already visited:
                                if (!closed.contains(n)) {
                                     closed.allocate(n, capacity, _params._maze.count());
                                     closed.emplace(n);

                                    // Create a parent node and transfer ownership of 'current' to it. Note: 'current' is now moved!
                                    open.emplace(n, &buf.emplace(std::move(curr)));
                                }
                            }
                        }
                    }
                }
                else { // SOLUTION REACHED ...

                    result = curr.template backtrack<bfs_node_t>(_params._maze.size(), capacity);

                    break;
                }
            }

            return result;
        }
    };

} //chdr::solvers

#endif //CHDR_BFS_HPP