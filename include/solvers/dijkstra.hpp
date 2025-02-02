/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_DIJKSTRA_HPP
#define CHDR_DIJKSTRA_HPP

/**
 * @file dijkstra.hpp
 */

#include <iostream>
#include <vector>

#include "base/solver.hpp"

// ReSharper disable once CppUnusedIncludeDirective
#include "../utils/intrinsics.hpp" // NOLINT(*-include-cleaner)

namespace chdr::solvers {

    /**
     * @struct dijkstra
     * @brief Dijkstra's algorithm [unfinished].
     * @details Dijkstra's algorithm (Dijkstra, E. W. 1959) is a graph traversal and pathfinding algorithm.
     *          It is a "single-source, multiple-target" (SSMT) algorithm, providing a technique for resolving
     *          the shortest path between a source and every other node in a graph.\n\n
     *
     * @warning This algorithm is not yet implemented.
     *
     * Advantages:
     * - Guarantees the lowest-cost path in graphs with non-negative edge weights.
     * - Effective and efficient in graphs with relatively few edges.
     * - Works well for both directed and undirected graphs.
     *
     * Limitations:
     * - Requires all edge weights to be non-negative (does not handle negative weights).
     * - Can be less efficient in dense graphs compared to other algorithms like Floyd-Warshall.
     * - Quickly consumes memory in large or exhaustive searches.
     * - Largely superseded by other algorithms for "single-source, single-target" (SSST) searches.
     *
     * Further Reading:
     * - <a href="https://en.wikipedia.org/wiki/Dijkstra's_algorithm">Wikipedia Article</a>
     *
     * References:
     * - Dijkstra, E. W., 1959. A Note on Two Problems in Connexion with Graphs. Numerische Mathematik, 1, 269–271.
     *
     * @note Unlike SSST algorithms, Dijkstra returns a shortest-path tree,
     *       indicating the shortest path between any node and the source.
     * @tparam params_t Type containing the search parameters.
     */
    template<typename params_t>
    struct [[maybe_unused]] dijkstra final {

        friend class solver<dijkstra, params_t>;

    private:

        using  index_t = typename params_t:: index_type;
        using scalar_t = typename params_t::scalar_type;
        using  coord_t = typename params_t:: coord_type;
        using solver_t = solver<dijkstra, params_t>;

        static_assert(std::is_integral_v<index_t>, "index_t must be an integral type.");

        [[maybe_unused, nodiscard]] HOT static constexpr auto invoke(const params_t& _params) {

            (void)_params;

            std::cerr << __PRETTY_FUNCTION__ << "Not implemented!\n";

            return std::vector<coord_t>{};
        }

    };

} //chdr::solvers

#endif //CHDR_DIJKSTRA_HPP