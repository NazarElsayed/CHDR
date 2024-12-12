/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_BSOLVER_HPP
#define CHDR_BSOLVER_HPP

#include "../../utils/intrinsics.hpp"

namespace chdr::solvers {

    template <
        template <size_t Kd, typename scalar_t, typename index_t, typename params_t> typename Derived,
        size_t Kd,
        typename scalar_t,
        typename  index_t,
        typename params_t
    >
    class solver final {

        using coord_t = coord<index_t, Kd>;

    public:

        [[maybe_unused, nodiscard]] constexpr auto operator() () {
            return operator()(params_t {});
        }

        template <typename... Args>
        [[maybe_unused, nodiscard]] constexpr auto operator() (Args&&... _args) {

            const params_t params { std::forward<Args>(_args)... };

            const auto s = utils::to_1d(params._start, params._size);
            const auto e = utils::to_1d(params._end,   params._size);

            if (params._maze.contains(s)       &&
                params._maze.contains(e)       &&
                params._maze.at(s).is_active() &&
                params._maze.at(e).is_active()
            ) {
                return s != e ?
                    Derived<Kd, scalar_t, index_t, params_t>::execute(params) :
                    std::vector<coord_t> { params._end };
            }
            else {
                return std::vector<coord_t>{};
            }
        }
    };

    template <
        template <size_t Kd, typename scalar_t, typename index_t, typename params_t> typename Derived,
        size_t Kd,
        typename scalar_t,
        typename index_t,
        typename params_t
    >
    [[nodiscard]] constexpr auto make_solver() {
        return solver<Derived, Kd, scalar_t, index_t, params_t>();
    }

    template <
        template <size_t Kd, typename scalar_t, typename index_t, typename params_t> typename Derived,
        size_t Kd,
        typename scalar_t,
        typename index_t,
        typename params_t
    >
    [[nodiscard]] constexpr auto solve() {
        return solver<Derived, Kd, scalar_t, index_t, params_t>()();
    }

    template <
        template <size_t Kd, typename scalar_t, typename index_t, typename params_t> typename Derived,
        size_t Kd,
        typename scalar_t,
        typename index_t,
        typename params_t,
        typename... Args
    >
    [[nodiscard]] constexpr auto solve(Args&&... _args) {
        return solver<Derived, Kd, scalar_t, index_t, params_t>()(std::forward<Args>(_args)...);
    }

} //chdr::solvers

#endif //CHDR_BSOLVER_HPP