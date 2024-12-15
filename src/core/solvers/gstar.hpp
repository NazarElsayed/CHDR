/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_GSTAR_HPP
#define CHDR_GSTAR_HPP

#include "mazes/graph.hpp"
#include "mazes/grid.hpp"
#include "mazes/base/imaze.hpp"
#include "types/existence_set.hpp"
#include "types/heap.hpp"
#include "utils/utils.hpp"

namespace chdr::solvers {

    template<size_t Kd, typename scalar_t, typename index_t, typename params_t>
    struct [[maybe_unused]] gstar final {

        friend struct solver<gstar, Kd, scalar_t, index_t, params_t>;

        static_assert(std::is_integral_v<scalar_t> || std::is_floating_point_v<scalar_t>, "scalar_t must be either an integral or floating point type");
        static_assert(std::is_integral_v<index_t>, "index_t must be an integral type.");

    private:

        using solver_t = solver<gstar, Kd, scalar_t, index_t, params_t>;
        using  coord_t = coord<index_t, Kd>;

        struct node final : managed_node<index_t> {

            scalar_t m_gScore;
            scalar_t m_fScore;

            [[nodiscard]] constexpr node() : managed_node<index_t>() {}

            [[nodiscard]] constexpr node(const index_t& _index, const scalar_t& _gScore, const scalar_t& _hScore) : managed_node<index_t>(_index),
                m_gScore(_gScore),
                m_fScore(_gScore + _hScore) {}

            [[nodiscard]] constexpr node(const index_t& _index, const scalar_t& _gScore, const scalar_t& _hScore, node&& _parent) : managed_node<index_t>(_index, std::move(_parent)),
                m_gScore(_gScore),
                m_fScore(_gScore + _hScore) {}

            [[nodiscard]] friend constexpr bool operator < (const node& _a, const node& _b) noexcept {
                return _a.m_fScore == _b.m_fScore ?
                       _a.m_gScore >  _b.m_gScore :
                       _a.m_fScore >  _b.m_fScore;
            }
        };

        using open_set_t = heap<node>;

        [[maybe_unused, nodiscard]] static constexpr std::vector<coord_t> execute(const params_t& _params) {

            std::vector<coord_t> result;

            const auto s = utils::to_1d(_params.start, _params.size);
            const auto e = utils::to_1d(_params.end,   _params.size);

            // Create closed set:
            const auto capacity = std::max(_params.capacity, std::max(s, e));
            existence_set closed({ s }, capacity);

            // Create open set:
            open_set_t open;
            open.emplace(s, static_cast<scalar_t>(0), _params.h(_params.start, _params.end));

            // Main loop:
            while (!open.empty()) {

                auto curr(std::move(open.top()));
                open.pop();

                if (curr.m_index != e) { // SEARCH FOR SOLUTION...

                    closed.allocate(curr.m_index, capacity, _params.maze.count());
                    closed.emplace(curr.m_index);

                    for (const auto& neighbour : _params.maze.get_neighbours(curr.m_index)) {

                        if constexpr (std::is_same_v<std::decay_t<decltype(_params.maze)>, mazes::graph<index_t, scalar_t>>) {

                            const auto& [n, nDistance] = neighbour;

                            const auto nCoord = utils::to_nd(n, _params.size);

                            // Check if node is not already visited:
                            if (!closed.contains(n)) {
                                 closed.allocate(n, capacity, _params.maze.count());
                                 closed.emplace(n);
                                   open.emplace(n, curr.m_gScore + nDistance, _params.h(nCoord, _params.end) * _params.weight, std::move(curr)); // Note: 'current' is now moved!
                            }
                        }
                        else {

                            if (const auto& [nActive, nCoord] = neighbour; nActive) {

                                const auto n = utils::to_1d(nCoord, _params.size);

                                constexpr scalar_t nDistance{1};

                                // Check if node is not already visited:
                                if (!closed.contains(n)) {
                                     closed.allocate(n, capacity, _params.maze.count());
                                     closed.emplace(n);
                                       open.emplace(n, curr.m_gScore + nDistance, _params.h(nCoord, _params.end) * _params.weight, std::move(curr)); // Note: 'current' is now moved!
                                }
                            }
                        }
                    }
                }
                else { // SOLUTION REACHED ...

                    // Free data which is no longer relevant:
                      open.clear();
                      open.shrink_to_fit();
                    closed.clear();
                    closed.shrink_to_fit();

                    result = curr.template backtrack<node>(_params.size, curr.m_gScore);

                    break;
                }
            }

            return result;
        }
    };

} //chdr::solvers

#endif //CHDR_GSTAR_HPP