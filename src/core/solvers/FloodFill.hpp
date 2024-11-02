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

#include "base/BSolver.hpp"
#include "mazes/base/IMaze.hpp"
#include "mazes/Graph.hpp"
#include "mazes/Grid.hpp"
#include "types/ExistenceSet.hpp"
#include "types/StableForwardBuf.hpp"
#include "utils/Utils.hpp"

namespace CHDR::Solvers {

    template<typename weight_t, const size_t Kd, typename scalar_t, typename index_t>
    class [[maybe_unused]] FloodFill final : BSolver<weight_t, Kd, scalar_t, index_t> {

        static_assert(std::is_integral_v<index_t>, "index_t must be an integral type.");

    private:

        using coord_t = Coord<index_t, Kd>;

    public:

        [[maybe_unused]]
        auto Solve(const Mazes::Graph<index_t, scalar_t>& _maze, const coord_t& _start, const coord_t& _end, const coord_t& _size, size_t _capacity = 0U) {

	        bool result = false;

            const auto s = Utils::To1D(_start, _size);
            const auto e = Utils::To1D(_end,   _size);

            if (_maze.Contains(s) &&
                _maze.Contains(e) &&
                _maze.At(s).IsActive() &&
                _maze.At(e).IsActive()
            ) {

                if (s != e) {

                    const auto count = _maze.Count();

                    std::queue<index_t> open;
                    open.emplace(s);

                    ExistenceSet closed ({ s }, std::max(_capacity, std::max(s, e)));

                    while (!open.empty()) {

                        for (size_t i = 0U; i < open.size(); ++i) {

                            const auto current(open.front());
                            open.pop();

                            if (current == e) {
                                result = true;

                                goto NestedBreak;
                            }

                            for (const auto& neighbour : _maze.GetNeighbours(current)) {

                                if (const auto& [nActive, nCoord] = neighbour; nActive) {

                                    const auto n = Utils::To1D(nCoord, _size);

                                    if (!closed.Contains(n)) {

                                        if (closed.Capacity() < n) {
                                            closed.Reserve(std::min(_capacity * ((n % _capacity) + 1U), count));
                                        }
                                        closed.Add(n);

                                        open.emplace(n);
                                    }
                                }
                            }
                        }
                    }
                }
                else {
                    result = true;
                }
            }

NestedBreak:
            return result;
        }

        [[maybe_unused]]
        auto Solve(const Mazes::Grid<Kd, weight_t>& _maze, const coord_t& _start, const coord_t& _end, size_t _capacity = 0U) {

	        bool result = false;

            const auto s = Utils::To1D(_start, _maze.Size());
            const auto e = Utils::To1D(_end,   _maze.Size());

            if (_maze.Contains(s) &&
                _maze.Contains(e) &&
                _maze.At(s).IsActive() &&
                _maze.At(e).IsActive()
            ) {

                if (s != e) {

                    const auto count = _maze.Count();

                    std::queue<index_t> open;
                    open.emplace(s);

                    ExistenceSet closed ({ s }, std::max(_capacity, std::max(s, e)));

                    while (!open.empty()) {

                        for (size_t i = 0U; i < open.size(); ++i) {

                            const auto current(open.front());
                            open.pop();

                            if (current == e) {
                                result = true;

                                goto NestedBreak;
                            }

                            for (const auto& neighbour : _maze.GetNeighbours(current)) {

                                if (const auto& [nActive, nCoord] = neighbour; nActive) {

                                    const auto n = Utils::To1D(nCoord, _maze.Size());

                                    if (!closed.Contains(n)) {

                                        if (closed.Capacity() < n) {
                                            closed.Reserve(std::min(_capacity * ((n % _capacity) + 1U), count));
                                        }
                                        closed.Add(n);
                                        
                                        open.emplace(n);
                                    }
                                }
                            }
                        }
                    }
                }
                else {
                    result = true;
                }
            }

NestedBreak:
            return result;
        }
    };

} // CHDR::Solvers

#endif //CHDR_FLOODFILL_HPP