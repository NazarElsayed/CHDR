/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_ISOLVER_HPP
#define CHDR_ISOLVER_HPP

#include "types/Heap.hpp"

namespace CHDR::Solvers {

    template<typename weight_t, const size_t Kd, typename scalar_t, typename index_t>
    class BSolver {

    private:
        using coord_t = Coord<index_t, Kd>;

    public:

        auto Solve(const Mazes::Grid<Kd, weight_t>& _maze, const coord_t& _start, const coord_t& _end, scalar_t (*_h)(const coord_t&, const coord_t&), const scalar_t& _weight = 1, size_t _capacity = 0U) const {

            std::vector<coord_t> result;

            const auto size  = _maze.Size();

            const auto s = Utils::To1D(_start, size);
            const auto e = Utils::To1D(_end  , size);

            if (_maze.Contains(s) &&
                _maze.Contains(e) &&
                _maze.At(s).IsActive() &&
                _maze.At(e).IsActive()
            ) {

            }

            return result;
        }
    };

} // CHDR::Solvers

#endif //CHDR_ISOLVER_HPP