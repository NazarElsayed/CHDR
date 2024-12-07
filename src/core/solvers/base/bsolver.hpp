/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_BSOLVER_HPP
#define CHDR_BSOLVER_HPP

namespace chdr::solvers {

    template<typename weight_t, const size_t Kd, typename scalar_t, typename index_t>
    class bsolver {

    private:

        using coord_t = coord<index_t, Kd>;

        virtual std::vector<coord_t> execute(
            const mazes::grid<Kd, weight_t>& _maze,
            const coord_t& _start,
            const coord_t& _end,
            scalar_t (*_h)(const coord_t&, const coord_t&),
            const scalar_t& _weight,
            const size_t _capacity
        ) const {

            (void)_maze;
            (void)_start;
            (void)_end;
            (void)_h;
            (void)_weight;
            (void)_capacity;

            throw std::runtime_error("Not implemented");
        }

        virtual std::vector<coord_t> execute(
            const mazes::graph<index_t, scalar_t>& _maze,
            const coord_t& _start,
            const coord_t& _end,
            const coord_t& _size,
            scalar_t (*_h)(const coord_t&, const coord_t&),
            const scalar_t& _weight,
            const size_t _capacity
        ) const {

            (void)_maze;
            (void)_start;
            (void)_end;
            (void)_size;
            (void)_h;
            (void)_weight;
            (void)_capacity;

            throw std::runtime_error("Not implemented");
        }

    public:

        virtual ~bsolver() = default;

        auto solve(
            const mazes::grid<Kd, weight_t>& _maze,
            const coord_t& _start,
            const coord_t& _end,
            scalar_t (*_h)(const coord_t&, const coord_t&) = nullptr,
            const scalar_t& _weight = 1,
            const size_t _capacity = 0U
        ) const {

            std::vector<coord_t> result;

            const auto size  = _maze.size();

            const auto s = utils::to_1d(_start, size);
            const auto e = utils::to_1d(_end  , size);

            if (_maze.contains(s) &&
                _maze.contains(e) &&
                _maze.at(s).is_active() &&
                _maze.at(e).is_active()
            ) {
                if (s != e) {
                    result = execute(_maze, _start, _end, _h, _weight, _capacity);
                }
                else {
                    result.emplace_back(_end);
                }
            }

            return result;
        }

        auto solve(
            const mazes::graph<index_t, scalar_t>& _maze,
            const coord_t& _start,
            const coord_t& _end,
            const coord_t& _size,
            scalar_t (*_h)(const coord_t&, const coord_t&) = nullptr,
            const scalar_t& _weight = 1,
            const size_t _capacity = 0U
        ) const {

            std::vector<coord_t> result;

            const auto s = utils::to_1d(_start, _size);
            const auto e = utils::to_1d(_end  , _size);

            if (_maze.contains(s) &&
                _maze.contains(e) &&
                    _maze.at(s).is_active() &&
                _maze.at(e).is_active()
            ) {
                if (s != e) {
                    result = execute(_maze, _start, _end, _size, _h, _weight, _capacity);
                }
                else {
                    result.emplace_back(_end);
                }
            }

            return result;
        }
    };

} //chdr::solvers

#endif //CHDR_BSOLVER_HPP