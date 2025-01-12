/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_SOLVER_HPP
#define CHDR_SOLVER_HPP

#include <cstddef>
#include <exception>
#include <type_traits>

namespace chdr::solvers {

    template <template <typename params_t> typename Derived, typename params_t>
    class solver final {

    public:

        template <typename T>
        struct is_graph : std::is_same<std::decay_t<T>, mazes::graph<typename params_t::index_type, typename params_t::scalar_type>>{};

        struct node_data {

            const bool active;

            const typename params_t:: coord_type coord;
            const typename params_t:: index_type index;
            const typename params_t::scalar_type distance;
        };

        template <typename maze_neighbour_t>
        static constexpr node_data get_data(const maze_neighbour_t& _n, const params_t& _params) noexcept {

            if constexpr (is_graph<decltype(_params.maze)>::value) {

                // _params.maze is a graph...
                const auto& [nIndex, nDistance] = _n;

                constexpr bool nActive = true;
                auto nCoord = utils::to_nd(nIndex, _params.size);

                return {
                    nActive,
                    nCoord,
                    nIndex,
                    nDistance
                };
            }
            else {

                // _params.maze is a grid...
                const auto& [nActive, nCoord] = _n;

                const auto nIndex = static_cast<typename params_t::index_type>(
                    nActive ? utils::to_1d(nCoord, _params.size) : typename params_t::index_type{}
                );

                constexpr auto nDistance = static_cast<typename params_t::scalar_type>(1);

                return {
                    nActive,
                    nCoord,
                    nIndex,
                    nDistance
                };
            }
        }

        static constexpr size_t determine_capacity(const params_t& _params) noexcept {

            if constexpr (is_graph<decltype(_params.maze)>::value) {

                return static_cast<typename params_t::index_type>(
                    _params.capacity != 0U ?
                        _params.capacity :
                        std::max(_params.maze.count() / 10U, static_cast<size_t>(1U))
                );
            }
            else {
                return std::max(
                    _params.capacity,
                    std::max(
                        static_cast<typename params_t::index_type>(utils::to_1d(_params.start, _params.size)),
                        static_cast<typename params_t::index_type>(utils::to_1d(_params.end,   _params.size))
                    )
                );
            }
        }

        [[maybe_unused, nodiscard]] constexpr auto operator()() const {
            return operator()(params_t {});
        }

        template <typename... Args>
        [[maybe_unused, nodiscard]]
#if __cplusplus >= 202003L
        constexpr
#endif // __cplusplus >= 202003L
        auto operator()(Args&&... _args) const {

            using solver_t = Derived<params_t>;

            const params_t params { std::forward<Args>(_args)... };

            const auto s = utils::to_1d(params.start, params.size);
            const auto e = utils::to_1d(params.end,   params.size);

            if (params.maze.contains(s) && params.maze.at(s).is_active() &&
                params.maze.contains(e) && params.maze.at(e).is_active()
            ) {
                return s != e ? solver_t::execute(params) : std::vector<typename params_t::coord_type> { params.end };
            }

            return std::vector<typename params_t::coord_type>{};
        }
    };

    template <template <typename params_t> typename Derived, typename params_t>
    [[nodiscard]] static
#if __cplusplus >= 2023L
    constexpr
#endif // __cplusplus >= 2023L
    auto make_solver() {
        return solver<Derived, params_t>();
    }

    template <template <typename params_t> typename Derived, typename params_t>
    [[nodiscard]] static
#if __cplusplus >= 2023L
    constexpr
#endif // __cplusplus >= 2023L
    auto solve() {
        return solver<Derived, params_t>()();
    }

    template <template <typename params_t> typename Derived, typename params_t, typename... Args>
    [[nodiscard]] static
#if __cplusplus >= 2023L
    constexpr
#endif // __cplusplus >= 2023L
    auto solve(Args&&... _args) {
        return solver<Derived, params_t>()(std::forward<Args>(_args)...);
    }

} //chdr::solvers

#endif //CHDR_SOLVER_HPP