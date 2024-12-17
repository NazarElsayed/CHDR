/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_BSOLVER_HPP
#define CHDR_BSOLVER_HPP

#include <cstddef>
#include <new>
#include <type_traits>

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

    private:

        template <typename C, typename = void>
        struct has_execute : std::false_type {};

        template <typename C>
        struct has_execute<C, std::void_t<decltype(std::declval<C>().execute(std::declval<params_t>()))>> : std::integral_constant<bool, true> {};

        // Check for `template<size_t> execute(params_t)`
        template <typename C, typename = void>
        struct has_templated_execute : std::false_type {};

        template <typename C>
        struct has_templated_execute<C, std::void_t<decltype(C::template execute<0U>(std::declval<params_t>()))>> : std::true_type {};

    public:

        template <typename T>
        struct is_graph : std::is_same<std::decay_t<T>, mazes::graph<index_t, scalar_t>> {};

        struct node_data {

            const bool active;

            const  coord_t    coord;
            const  index_t    index;
            const scalar_t distance;
        };

        template <typename maze_neighbour_t>
        static constexpr node_data get_data(const maze_neighbour_t& _n, const params_t& _params) {

            if constexpr (std::is_same_v<std::decay_t<decltype(_params.maze)>, mazes::graph<index_t, scalar_t>>) {

                // _params.maze is graph...

                const auto& [nIndex, nDistance] = _n;

                return {
                    true,
                    utils::to_nd(nIndex, _params.size),
                    nIndex,
                    nDistance
                };
            }
            else {

                // _params.maze is grid...

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
        [[maybe_unused, nodiscard]] auto operator()(Args&&... _args) const {

            using solver_t = Derived<Kd, scalar_t, index_t, params_t>;

            const params_t params { std::forward<Args>(_args)... };

            try {
                
                const auto s = utils::to_1d(params.start, params.size);
                const auto e = utils::to_1d(params.end,   params.size);

                if (params.maze.contains(s)       &&
                    params.maze.contains(e)       &&
                    params.maze.at(s).is_active() &&
                    params.maze.at(e).is_active()
                ) {
                    return s != e ? solver_t::execute(params) : std::vector<coord_t> { params.end };
                }
            }
            catch (const std::bad_alloc& e) {
                std::cerr << "[std::bad_alloc] (solver::operator()): " << e.what() << "\n";
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