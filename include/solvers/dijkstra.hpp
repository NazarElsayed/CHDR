/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_DIJKSTRA_HPP
#define CHDR_DIJKSTRA_HPP

#include <cstddef>
#include <vector>

#include "base/solver.hpp"

namespace chdr::solvers {

    template<typename params_t>
    struct [[maybe_unused]] dijkstra final {

        friend class solver<dijkstra, params_t>;

    private:

        using  index_t = typename params_t:: index_type;
        using scalar_t = typename params_t::scalar_type;
        using  coord_t = typename params_t:: coord_type;
        using solver_t = solver<dijkstra, params_t>;

        static_assert(std::is_integral_v<index_t>, "index_t must be an integral type.");

        [[maybe_unused, nodiscard]] static auto execute(const params_t& _params) {

            (void)_params;

            throw std::runtime_error("Djikstra::solve(const Mazes::Grid<Kd, weight_t>& _maze, const coord_t& _start, const coord_t& _end, scalar_t (*_h)(const coord_t&, const coord_t&), size_t _capacity = 0U): Not implemented!");
        }

    };

} //chdr::solvers

#endif //CHDR_DIJKSTRA_HPP