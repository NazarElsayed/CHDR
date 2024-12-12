/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_FLOODFILL_HPP
#define CHDR_FLOODFILL_HPP

#include "mazes/graph.hpp"
#include "mazes/grid.hpp"
#include "mazes/base/imaze.hpp"
#include "types/existence_set.hpp"
#include "utils/utils.hpp"

namespace chdr::solvers {

    template<typename weight_t, const size_t Kd, typename scalar_t, typename index_t, typename params_t>
    class [[maybe_unused]] floodfill final {

        static_assert(std::is_integral_v<index_t>, "index_t must be an integral type.");

    private:

        using coord_t = coord<index_t, Kd>;

    public:

        [[maybe_unused]]
        auto solve(const mazes::graph<index_t, scalar_t>& _maze, const coord_t& _start, const coord_t& _end, const coord_t& _size, size_t _capacity = 0U) {

	        bool result = false;

            const auto s = utils::to_1d(_start, _size);
            const auto e = utils::to_1d(_end,   _size);

            if (_maze.contains(s) &&
                _maze.contains(e) &&
                _maze.at(s).is_active() &&
                _maze.at(e).is_active()
            ) {

                if (s != e) {

                    // Create closed Set:
                    existence_set closed ({ s }, std::max(_capacity, std::max(s, e)));

                    // Create open set:
                    chdr::queue<index_t> open;
                    open.emplace(s);

                    // Main loop:
                    while (!open.empty() && !result) {

                        for (size_t i = 0U; i < open.size(); ++i) {

                            const auto curr(std::move(open.front()));
                            open.pop();

                            if (curr != e) {

                                closed.allocate(curr.m_index, _capacity, _maze.count());
                                closed.emplace(curr.m_index);

                                for (const auto& neighbour : _maze.get_neighbours(curr)) {

                                    if (const auto& [nActive, nCoord] = neighbour; nActive) {

                                        const auto n = utils::to_1d(nCoord, _maze.size());

                                        if (!closed.contains(n)) {

                                            closed.allocate(n, _capacity, _maze.count());
                                            closed.emplace(n);

                                            open.emplace(n);
                                        }
                                    }
                                }
                            }
                            else {
                                result = true;

                                break;
                            }
                        }
                    }
                }
                else {
                    result = true;
                }
            }

            return result;
        }

        [[maybe_unused]]
        auto solve(const mazes::grid<Kd, weight_t>& _maze, const coord_t& _start, const coord_t& _end, size_t _capacity = 0U) {

	        bool result = false;

            const auto s = utils::to_1d(_start, _maze.size());
            const auto e = utils::to_1d(_end,   _maze.size());

            if (_maze.contains(s) &&
                _maze.contains(e) &&
                _maze.at(s).is_active() &&
                _maze.at(e).is_active()
            ) {

                if (s != e) {

                    // Create closed set:
                    existence_set closed ({ s }, std::max(_capacity, std::max(s, e)));

                    // Create open set:
                    chdr::queue<index_t> open;
                    open.emplace(s);

                    // Main loop:
                    while (!open.empty() && !result) {

                        for (size_t i = 0U; i < open.size(); ++i) {

                            const auto curr(std::move(open.front()));
                            open.pop();

                            if (curr != e) {

                                closed.allocate(curr.m_index, _capacity, _maze.count());
                                closed.emplace(curr.m_index);

                                for (const auto& neighbour : _maze.get_neighbours(curr)) {

                                    if (const auto& [nActive, nCoord] = neighbour; nActive) {

                                        const auto n = utils::to_1d(nCoord, _maze.size());

                                        if (!closed.contains(n)) {

                                            closed.allocate(n, _capacity, _maze.count());
                                            closed.emplace(n);

                                            open.emplace(n);
                                        }
                                    }
                                }
                            }
                            else {
                                result = true;

                                break;
                            }
                        }
                    }
                }
                else {
                    result = true;
                }
            }

            return result;
        }
    };

} //chdr::solvers

#endif //CHDR_FLOODFILL_HPP