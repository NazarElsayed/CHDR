/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_BSOLVER_HPP
#define CHDR_BSOLVER_HPP
#include "types/existence_set.hpp"
#include "types/stable_forward_buf.hpp"

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

        struct node_data {

            const bool active;

            const  coord_t    coord;
            const  index_t    index;
            const scalar_t distance;
        };

        template <typename maze_neighbour_t>
        static constexpr node_data get_data(const maze_neighbour_t& _n, const params_t& _params) {

            constexpr bool is_graph = std::is_same_v<std::decay_t<decltype(_params.maze)>, mazes::graph<index_t, scalar_t>>;

            if constexpr (is_graph) {

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

        [[maybe_unused, nodiscard]] constexpr auto operator()() const {
            return operator()(params_t {});
        }

        template <typename... Args>
        [[maybe_unused, nodiscard]] constexpr auto operator()(Args&&... _args) const {

            const params_t params { std::forward<Args>(_args)... };

            const auto s = utils::to_1d(params.start, params.size);
            const auto e = utils::to_1d(params.end,   params.size);

            if (params.maze.contains(s)       &&
                params.maze.contains(e)       &&
                params.maze.at(s).is_active() &&
                params.maze.at(e).is_active()
            ) {
                return s != e ?
                    Derived<Kd, scalar_t, index_t, params_t>::execute(params) :
                    std::vector<coord_t> { params.end };
            }

            return std::vector<coord_t>{};
        }
    };

    template <
        template <size_t Kd, typename scalar_t, typename index_t, typename params_t> typename Derived,
        size_t Kd,
        typename scalar_t,
        typename index_t,
        typename params_t
    >
    [[nodiscard]] static constexpr auto make_solver() {
        return solver<Derived, Kd, scalar_t, index_t, params_t>();
    }

    template <
        template <size_t Kd, typename scalar_t, typename index_t, typename params_t> typename Derived,
        size_t Kd,
        typename scalar_t,
        typename index_t,
        typename params_t
    >
    [[nodiscard]] static constexpr auto solve() {
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
    [[nodiscard]] static constexpr auto solve(Args&&... _args) {
        return solver<Derived, Kd, scalar_t, index_t, params_t>()(std::forward<Args>(_args)...);
    }

} //chdr::solvers

#endif //CHDR_BSOLVER_HPP