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
        using       node = unmanaged_node<index_t>;
        using open_set_t = queue<node>;

        struct node_data {

            const bool active;

            const  coord_t    coord;
            const  index_t    index;
            const scalar_t distance;
        };

        template <typename node_data_t>
        static constexpr node_data get_data(const node_data_t& _n, const params_t& _params) {

            constexpr bool is_graph = std::is_same_v<std::decay_t<decltype(_params.maze)>, mazes::graph<index_t, scalar_t>>;

            if constexpr(is_graph) {

                const auto& [nIndex, nDistance] = _n;

                return {
                    true,
                    utils::to_nd(nIndex, _params.size),
                    nIndex,
                    nDistance
                };
            }
            else {

                const auto& [nActive, nCoord] = _n;

                return {
                    nActive,
                    nCoord,
                    nActive ? utils::to_1d(nCoord, _params.size) : index_t{},
                    static_cast<scalar_t>(1)
                };
            }
        }

        [[maybe_unused, nodiscard]] static constexpr std::vector<coord_t> execute(const params_t& _params) {

            std::vector<coord_t> result;

            const auto s = utils::to_1d(_params.start, _params.size);
            const auto e = utils::to_1d(_params.end,   _params.size);

            // Create closed set:
            const auto capacity = std::max(_params.capacity, std::max(s, e));
            existence_set<low_memory_usage> closed({ s }, capacity);

            // Create open:
            open_set_t open;
            open.emplace(s);

            // Create buffer:
            stable_forward_buf<node> buf;

            // Main loop:
            while (!open.empty()) { // SEARCH FOR SOLUTION...

                auto curr(std::move(open.top()));
                open.pop();

                if (curr.m_index != e) {

                    closed.allocate(curr.m_index, capacity, _params.maze.count());
                    closed.emplace(curr.m_index);

                    for (const auto& n_data : _params.maze.get_neighbours(curr.m_index)) {

                        if (const auto n = get_data(n_data, _params); n.active) {

                            // Check if node is not already visited:
                            if (!closed.contains(n.index)) {
                                 closed.allocate(n.index, capacity, _params.maze.count());
                                 closed.emplace(n.index);
                                   open.emplace(n.index, &buf.emplace(std::move(curr))); // Note: 'current' is now moved!
                            }
                        }
                    }
                }
                else { // SOLUTION REACHED ...

                    result = curr.template backtrack<node>(_params.size, capacity);

                    break;
                }
            }

            return result;
        }
    };

} //chdr::solvers

#endif //CHDR_BFS_HPP