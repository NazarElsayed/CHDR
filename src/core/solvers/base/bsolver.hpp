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

    template<typename weight_t, const size_t Kd, typename scalar_t, typename index_t, typename params_t>
    class bsolver {

    private:

        using coord_t = coord<index_t, Kd>;

       [[nodiscard]] virtual std::vector<coord_t> execute(const params_t& _params) const {

            (void)_params;

            throw std::runtime_error("Not implemented");
        }

    public:

        virtual ~bsolver() = default;

        [[maybe_unused, nodiscard]] auto solve(const params_t& _params) const {

            const auto s = utils::to_1d(_params._start, _params._maze.size());
            const auto e = utils::to_1d(_params._end  , _params._maze.size());

            if (_params._maze.contains(s)       &&
                _params._maze.contains(e)       &&
                _params._maze.at(s).is_active() &&
                _params._maze.at(e).is_active()
            ) {
                return (s != e) ? execute(_params) : std::vector<coord_t> { _params._end };
            }
            else {
                return std::vector<coord_t> {};
            }
        }
    };

} //chdr::solvers

#endif //CHDR_BSOLVER_HPP