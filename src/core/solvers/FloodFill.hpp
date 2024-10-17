/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_FLOODFILL_HPP
#define CHDR_FLOODFILL_HPP

#include <queue>

#include "base/ISolver.hpp"
#include "mazes/base/IMaze.hpp"
#include "mazes/Grid.hpp"
#include "types/ExistenceSet.hpp"
#include "utils/Utils.hpp"

namespace CHDR::Solvers {

    template<typename Tm, const size_t Kd>
    class FloodFill final {

    private:

        using coord_t = Coord<size_t, Kd>;

    public:

        auto Solve(const Mazes::Grid<Kd, Tm>& _maze, const coord_t& _start, const coord_t& _end, size_t _capacity = 0U) {

	        bool result = false;

            const auto s = Utils::To1D(_start, _maze.Size());
            const auto e = Utils::To1D(_end,   _maze.Size());

            if (_maze.Contains(s) &&
                _maze.Contains(e) &&
                _maze.At(s).IsActive() &&
                _maze.At(e).IsActive()
            ) {

                const auto maze_count = _maze.Count();

                std::queue<size_t> openSet;
                openSet.emplace(s);

                ExistenceSet closedSet ({ s }, std::max(_capacity, std::max(s, e)));

                while (!openSet.empty()) {

                    for (size_t i = 0U; i < openSet.size(); ++i) {

                        const auto current = openSet.front();
                        openSet.pop();

                        if (current == e) {
                            result = true;

                            goto NestedBreak;
                        }

                        for (const auto& neighbour : _maze.GetNeighbours(current)) {

                            if (const auto& [nActive, nCoord] = neighbour; nActive) {

                                const auto n = Utils::To1D(nCoord, _maze.Size());

                                if (!closedSet.Contains(n)) {

                                    if (closedSet.Capacity() > n) {
                                        closedSet.Reserve(std::min(_capacity * ((n % _capacity) + 1U), maze_count));
                                    }
                                    closedSet.Add(n);

                                    openSet.emplace(n);
                                }
                            }
                        }
                    }
                }
            }

NestedBreak:
            return result;
        }
    };

} // CHDR::Solvers

#endif //CHDR_FLOODFILL_HPP