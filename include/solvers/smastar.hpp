/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_SMASTAR_HPP
#define CHDR_SMASTAR_HPP

/**
 * @file smastar.hpp
 */

#include <cstddef>
#include <set>
#include <type_traits>
#include <vector>

#include "../utils/utils.hpp"
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
     * @addtogroup SingleTargetCommon Common
     * @brief General-purpose solvers.
     * @{
     */

    /**
     * @struct smastar
     * @brief SMA* search algorithm.
     *
     * @tparam params_t Type containing the search parameters.
     */
    template<typename params_t>
    struct [[maybe_unused]] smastar final {

        friend class solver<smastar, params_t>;

    private:

        using  coord_t = typename params_t:: coord_type;
        using  index_t = typename params_t:: index_type;
        using scalar_t = typename params_t::scalar_type;
        using solver_t = solver<smastar, params_t>;
        using weight_t = typename params_t::weight_type;

        static_assert(std::is_arithmetic_v<scalar_t>, "scalar_t must be an integral or floating point type.");
        static_assert(std::is_integral_v<index_t>, "index_t must be an integral type.");

        static constexpr auto null_v = std::numeric_limits< index_t>::max();
        static constexpr auto  inf_v = std::numeric_limits<scalar_t>::max();

        static constexpr auto Kd = std::tuple_size_v<std::decay_t<typename params_t::coord_type>>;

        struct node final {

             index_t m_index;
            scalar_t m_gScore;
            scalar_t m_fScore;
             index_t m_parent;

            /**
             * @brief Constructs an uninitialized node.
             */
            // ReSharper disable once CppPossiblyUninitializedMember
            [[nodiscard]] HOT constexpr node() noexcept {} // NOLINT(*-pro-type-member-init, *-use-equals-default)

            [[nodiscard]] HOT constexpr node(index_t _index, scalar_t _gScore, scalar_t _fScore, index_t _parent = null_v) noexcept :
                m_index     (_index ),
                m_gScore    (_gScore),
                m_fScore    (_fScore),
                m_parent    (_parent) {}

            ~node() = default;

            node           (const node&) = default;
            node& operator=(const node&) = default;

            [[nodiscard]] HOT constexpr node(node&& _other) noexcept = default;

#if __cplusplus > 202302L
            constexpr
#endif
            HOT node& operator=(node&& _other) noexcept = default;

            [[nodiscard]] HOT friend constexpr bool operator < (const node& _a, const node& _b) noexcept {
                return _a.m_fScore == _b.m_fScore ?
                       _a.m_gScore >  _b.m_gScore :
                       _a.m_fScore <  _b.m_fScore;
            }
        };

        template<typename open_set_t, typename all_nodes_t>
        HOT static void remove_worst(open_set_t& _open, all_nodes_t& _all_nodes, const params_t& _params) {
            assert(!_open.empty() && "_open cannot be empty.");

            auto worst_node = std::prev(_open.end());
            backup_f_values(*worst_node, _all_nodes, _params);
            _all_nodes.erase(worst_node->m_index);
                 _open.erase(worst_node);
        }

        template <typename all_nodes_t>
        HOT static void backup_f_values(const node& _removed_node, all_nodes_t& _all_nodes, const params_t& _params) {

            if (auto p_index = _removed_node.m_parent; p_index != null_v) {

                for (auto p = _all_nodes.find(p_index); p != _all_nodes.end(); p_index = p->second.m_parent) {

                    auto min_f = inf_v;

                    for (const auto& n_data : _params.maze.get_neighbours(p->second.m_index)) {

                        if (const auto& n = solver_t::get_data(n_data, _params); n.active) {

                            if (auto child_it = _all_nodes.find(n.index); child_it != _all_nodes.end()) {
                                min_f = std::min(min_f, child_it->second.m_fScore);
                            }
                        }
                    }

                    // If updating the parent's f-value is necessary:
                    if (min_f > p->second.m_fScore) {
                        p->second.m_fScore = min_f;
                    }
                    else {
                        break;
                    }
                }
            }
        }

        template<typename all_nodes_t>
        HOT static auto rbacktrack(const node& _curr, const all_nodes_t& _all_nodes, const params_t& _params) {

            /*
             * Warnings regarding potential null-pointer dereferencing are paranoid in this situation.
             */

#if defined(__GNUC__) || defined(__clang__) // __GNUC__ || __clang__
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wnull-dereference"
#elif defined(_MSC_VER) // !(__GNUC__ || __clang__) && _MSC_VER
    #pragma warning(push)
    #pragma warning(disable: 6011)
#endif // !(__GNUC__ || __clang__) && _MSC_VER

            std::vector<coord_t> result{};

            /*
             * Determine the size of the final path prior to backtracking.
             * If the maze is a graph, then the size must be determined through recursion.
             */

            if constexpr (solver_t::solver_utils::template is_graph_v<decltype(_params.maze)>) { // GRAPH...

                // Solution depth obtained using recursion...
                size_t depth = 0U;
                for (auto p = _curr.m_parent; p != null_v; p = _all_nodes.find(p)->second.m_parent) {
                    ++depth;
                }

                result.resize(depth);
            }
            else { // GRID...

                // Uniform space; solution depth can be calculated from g-score.
                result.resize(static_cast<size_t>(_curr.m_gScore));
            }

            size_t i = 0U;
            for (auto p = _curr.m_parent; p != null_v; p = _all_nodes.find(p)->second.m_parent, ++i) {
                result[(result.size() - 1U) - i] = utils::to_nd(p, _params.size);
            }

#if defined(__GNUC__) || defined(__clang__) // __GNUC__ || __clang__
    #pragma GCC diagnostic pop
#elif defined(_MSC_VER) // !(__GNUC__ || __clang__) && _MSC_VER
    #pragma warning(pop)
#endif // !(__GNUC__ || __clang__) && _MSC_VER

            return result;
        }

        template <typename open_set_t>
        [[nodiscard]] HOT static constexpr auto solve_internal(open_set_t& _open, const params_t& _params) {

            const auto s = utils::to_1d(_params.start, _params.size);
            const auto e = utils::to_1d(_params.end,   _params.size);

            std::unordered_map<index_t, node> all_nodes;

#if CHDR_DIAGNOSTICS == 1
            size_t peak_memory_usage = 0U;
#endif //CHDR_DIAGNOSTICS == 1

            _open.emplace(
                all_nodes[static_cast<index_t>(s)] = node(
                    static_cast< index_t>(s),
                    static_cast<scalar_t>(0),
                    _params.h(_params.start, _params.end) * _params.weight
                )
            );

            while (!_open.empty()) {

                auto curr = _open.extract(_open.begin()).value();

                if (curr.m_index == e) { // SOLUTION FOUND...

#if CHDR_DIAGNOSTICS == 1
                    std::cout << "Peak Memory Usage: " << peak_memory_usage << "\n";
#endif //CHDR_DIAGNOSTICS == 1

                    return rbacktrack(curr, all_nodes, _params);
                }
                else { // SEARCH FOR SOLUTION...

                    bool has_neighbours = false;

                    if (curr.m_fScore != inf_v) {

                        for (const auto& n_data : _params.maze.get_neighbours(curr.m_index)) {

                            if (const auto& n = solver_t::get_data(n_data, _params); n.active) {

                                // Check if the neighbour 'exists', or update its parent if the current route is better.
                                auto search = all_nodes.find(n.index);
                                if (search == all_nodes.end()) {

                                    has_neighbours = true;

                                    // Attempt to make room for the new node:
                                    if (!_open.empty() && all_nodes.size() >= _params.memory_limit) {
                                        remove_worst(_open, all_nodes, _params);
                                    }

                                    // Only instantiate if there is room in memory. Otherwise, break.
                                    if (all_nodes.size() < _params.memory_limit) {

                                        const auto g = curr.m_gScore + n.distance;
                                        const auto h = _params.h(n.coord, _params.end) * _params.weight;

                                        _open.emplace(all_nodes[n.index] = node(n.index, g, g + h, curr.m_index));

#if CHDR_DIAGNOSTICS == 1
                                        peak_memory_usage = utils::max(peak_memory_usage, all_nodes.size());
#endif //CHDR_DIAGNOSTICS == 1
                                    }
                                    else {
                                        break;
                                    }
                                }
                                else if (search->second.m_fScore != inf_v) {

                                    if (const auto g = curr.m_gScore + n.distance; g < search->second.m_gScore) {
                                        const auto h = _params.h(n.coord, _params.end) * _params.weight;

                                        _open.erase(search->second);
                                        _open.emplace(all_nodes[n.index] = node(n.index, g, g + h, curr.m_index));

                                        has_neighbours = true;
                                    }
                                }
                            }
                        }
                    }

                    if (UNLIKELY(!has_neighbours)) { // DEAD END...
                        curr.m_fScore = inf_v;
                        backup_f_values(curr, all_nodes, _params);
                        all_nodes[curr.m_index] = curr;
                    }
                }
            }

#if CHDR_DIAGNOSTICS == 1
            std::cout << "Peak Memory Usage: " << peak_memory_usage << "\n";
#endif //CHDR_DIAGNOSTICS == 1

            return std::vector<coord_t>{};
        }

        [[maybe_unused, nodiscard]] static auto invoke(const params_t& _params) {

            if (_params.memory_limit > 0U) {

                std::pmr::multiset<node> _open(_params.homogeneous_pmr);

                return solve_internal(_open, _params);
            }
            else {
                return std::vector<coord_t>{};
            }
        }
    };

    /**
     * @}
     * @}
     * @}
     */

} //chdr::solvers

#endif //CHDR_ASTAR_HPP