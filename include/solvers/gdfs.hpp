/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_GDFS_HPP
#define CHDR_GDFS_HPP

/**
 * @file gdfs.hpp
 */

#include <cstddef>
#include <vector>

#include "../types/containers/existence_set.hpp"
#include "../types/containers/stack.hpp"
#include "../utils/utils.hpp"
#include "base/managed_node.hpp"
#include "base/solver.hpp"

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
     * @addtogroup SingleTargetGraveyardOptimised Graveyard-Optimised
     * @brief Graveyard solvers, which dynamically prune the search tree.
     * @{
     */

    /**
     * @struct gdfs
     * @brief Graveyard-optimised variant of the depth-first search algorithm.
     * @details G*-optimised variant of DFS, reducing memory usage by allowing for the dynamic
     *          pruning of redundant data from the search tree.
     *
     * Advantages:
     * - Significantly reduced average memory usage when compared to DFS.
     * - Capable of outperforming DFS if the overhead due to memory allocation is lower.
     *
     * Limitations:
     * - Higher constant factor than DFS makes it less effective in small searches.
     *
     * @tparam params_t Type containing the search parameters.
     * @see dfs
     * @see gstar
     */
    template<typename params_t>
    struct [[maybe_unused]] gdfs final {

        friend class solver<gdfs, params_t>;

    private:

        using  index_t = typename params_t::index_type;
        using  coord_t = typename params_t::coord_type;
        using solver_t = solver<gdfs, params_t>;
        using     node = managed_node<index_t>;

        template <typename open_set_t, typename closed_set_t>
        [[nodiscard]] HOT static constexpr auto solve_internal(open_set_t& _open, closed_set_t& _closed, size_t _capacity, const params_t& _params) {

            const auto s = utils::to_1d(_params.start, _params.size);
            const auto e = utils::to_1d(_params.end,   _params.size);

              _open.emplace(s);
            _closed.emplace(s);

            // Main loop:
            while (LIKELY(!_open.empty())) { // SEARCH FOR SOLUTION...

                auto curr(std::move(_open.front()));
                _open.pop();

                if (curr.m_index != e) { // SEARCH FOR SOLUTION...

                    node* RESTRICT curr_ptr(nullptr);

                    for (const auto& n_data: _params.maze.get_neighbours(curr.m_index)) {

                        if (const auto& n = solver_t::get_data(n_data, _params); n.active) {

                            // Check if node is not already visited:
                            if (!_closed.contains(n.index)) {
                                solver_t::solver_utils::preallocate_emplace(_closed, n.index, _capacity, _params.maze.count());

                                if (curr_ptr == nullptr) {
                                    curr_ptr = new (_params.homogeneous_pmr->allocate(sizeof(node), alignof(node))) node(std::move(curr));
                                }

                                _open.emplace(n.index, curr_ptr);
                            }
                        }
                    }

                    if (curr_ptr == nullptr) {
                        curr.expunge(_params.homogeneous_pmr);
                    }
                }
                else { // SOLUTION REACHED...

                    _open   =   open_set_t{};
                    _closed = closed_set_t{};

                    return solver_t::solver_utils::rbacktrack(curr, _params.size);
                }
            }

            return std::vector<coord_t>{};
        }

        [[maybe_unused, nodiscard]] static auto invoke(const params_t& _params) {

            const auto capacity = solver_t::solver_utils::determine_capacity(_params);

            existence_set closed(_params.monotonic_pmr);
            closed.reserve(capacity);

            stack<node> open(_params.heterogeneous_pmr);

            return solve_internal(open, closed, capacity, _params);
        }
    };

    /**
     * @}
     * @}
     * @}
     */

} //chdr::solvers

#endif //CHDR_GDFS_HPP