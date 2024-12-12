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

        template <typename... Args>
        [[maybe_unused, nodiscard]] constexpr auto operator() (Args&&... _args) {

            const params_t params { std::forward<Args>(_args)... };

            const auto s = utils::to_1d(params._start, params._maze.size());
            const auto e = utils::to_1d(params._end  , params._maze.size());

            if (params._maze.contains(s)       &&
                params._maze.contains(e)       &&
                params._maze.at(s).is_active() &&
                params._maze.at(e).is_active()
            ) {
                return (s != e) ? execute(params) : std::vector<coord_t> { params._end };
            }
            else {
                return std::vector<coord_t> {};
            }
        }
    };

} //chdr::solvers

#endif //CHDR_BSOLVER_HPP