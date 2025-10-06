/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_BFS_HPP
#define CHDR_BFS_HPP

/**
 * @file bfs.hpp
 */

#include <cstddef>
#include <vector>

#include "../types/containers/existence_set.hpp"
#include "../types/containers/queue.hpp"
#include "../utils/utils.hpp"
#include "base/solver.hpp"
#include "base/unmanaged_node.hpp"

// ReSharper disable once CppUnusedIncludeDirective
#include "../utils/intrinsics.hpp" // NOLINT(*-include-cleaner)

namespace chdr::solvers {

    /**
     * @addtogroup Solvers
     * @brief Graph traversal and pathfinding algorithms.
     * @{
     * @addtogroup Single-Target
     * @brief Solvers which route to a single destination.
     * @{
     * @addtogroup SingleTargetCommon Common
     * @brief General-purpose solvers.
     * @{
     */

    /**
     * @struct bfs
     * @brief Breadth-first search algorithm.
     * @details A graph traversal and pathfinding algorithm that expands every search neighbour during an iteration.\n\n
     *
     * Advantages:
     * - Low constant time factor.
     * - Lower constant memory factor than most heuristic-informed algorithms.
     * - Effective in searches with many obstacles.
     * - Well suited for solvable problems in unbounded (infinite) space.
     * - Does not need a pre-pass, although performance can improve if the search space is pruned first.
     * - The resulting path is optimal in uniform-cost graphs.
     *
     * Limitations:
     * - Does not always guarantee an optimal path if the search space is not uniform.
     * - Quickly consumes memory in large or exhaustive searches.
     * - Often slower than heuristic-informed searches.
     *
     * Further Reading:
     * - <a href="https://en.wikipedia.org/wiki/Breadth-first_search">Wikipedia Article</a>
     *
     * @note If the search space is uniform-cost, the resulting path is optimal.
     *
     * @tparam params_t Type containing the search parameters.
     */
    template<typename params_t>
    struct [[maybe_unused]] bfs final {

        friend class solver<bfs, params_t>;

    private:

        using  index_t = typename params_t::index_type;
        using  coord_t = typename params_t::coord_type;
        using solver_t = solver<bfs, params_t>;
        using     node = unmanaged_node<index_t>;

        template <typename open_set_t, typename closed_set_t>
        [[nodiscard]] HOT static constexpr auto solve_internal(open_set_t& _open, closed_set_t& _closed, size_t _capacity, const params_t& _params) {

            constexpr bool do_reverse = !solver_t::solver_utils::template is_graph_v<decltype(_params.maze)> && params_t::reverse_equivalence::value;

            const auto&   end = do_reverse ? _params.start : _params.end;
            const auto& start = do_reverse ? _params.end   : _params.start;

            const auto s = utils::to_1d(start, _params.size);
            const auto e = utils::to_1d(end,   _params.size);

              _open.emplace(s);
            _closed.emplace(s);

            // Main loop:
            while (LIKELY(!_open.empty())) {

                auto curr(std::move(_open.top()));
                _open.pop();

                if (curr.m_index != e) { // SEARCH FOR SOLUTION...

                    node* RESTRICT curr_ptr(nullptr);

                    for (const auto& n_data : _params.maze.template get_neighbours<params_t::octile_neighbours::value>(curr.m_index)) {

                        if (const auto& n = solver_t::get_data(n_data, _params); n.active) {

                            // Check if node is not already visited:
                            if (!_closed.contains(n.index)) {
                                solver_t::solver_utils::preallocate_emplace(_closed, n.index, _capacity, _params.maze.count());

                                if (curr_ptr == nullptr) {
                                    curr_ptr = new (_params.monotonic_pmr->allocate(sizeof(node), alignof(node))) node(std::move(curr));
                                }

                                _open.emplace(n.index, curr_ptr);
                            }
                        }
                    }
                }
                else { // SOLUTION REACHED...

                    _open   =   open_set_t{};
                    _closed = closed_set_t{};

                    if constexpr (params_t::reverse_equivalence::value) {
                        return solver_t::solver_utils::rbacktrack(curr, _params.size);
                    }
                    else {
                        return solver_t::solver_utils::rbacktrack_noreverse(curr, _params.size);
                    }
                }
            }

            return std::vector<coord_t>{};
        }

        [[maybe_unused, nodiscard]] static auto invoke(const params_t& _params) {

            const auto capacity = solver_t::solver_utils::determine_capacity(_params);

            existence_set closed(_params.monotonic_pmr);
            closed.reserve(capacity);

            queue<node> open(_params.heterogeneous_pmr);

            return solve_internal(open, closed, capacity, _params);
        }
    };

    /**
     * @}
     * @}
     * @}
     */

} //chdr::solvers

#endif //CHDR_BFS_HPP