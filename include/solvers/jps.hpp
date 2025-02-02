/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_JPS_HPP
#define CHDR_JPS_HPP

/**
 * @file jps.hpp
 */

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <utility>

#include "../mazes/grid.hpp"
#include "../types/containers/existence_set.hpp"
#include "../utils/utils.hpp"
#include "base/solver.hpp"

// ReSharper disable once CppUnusedIncludeDirective
#include "../utils/intrinsics.hpp" // NOLINT(*-include-cleaner)

namespace chdr::solvers {

    /**
     * @nosubgrouping
     * @class jps
     * @brief Jump-point search algorithm.
     * @details JPS (Harabor, D. and Grastien, A., 2011) is a pathfinding algorithm for optimal routing through
     *          uniform-cost grids.\n
     *          JPS employs symmetry-breaking techniques to avoid processing nodes that do not contribute to the search,
     *          allowing for significantly reduced search times and memory usage.\n\n
     *
     * Advantages:
     * - Faster average search times than A*.
     * - Lower average memory usage than A*.
     *
     * Limitations:
     * - Limited to searches on uniform-cost grid topologies.
     * - Less effective than A* when the search space cannot be reduced.
     * - Higher constant factor than A* makes JPS slower in small search spaces.
     * - Inappropriate for use in unbounded (infinite) search spaces.
     * - Inefficient or complex search heuristics can reduce performance.
     * - Poor performance when searches lack solutions.
     *
     * Further Reading:
     * - <a href="https://en.wikipedia.org/wiki/Jump_point_search">Wikipedia Article</a>
     * - <a href="https://www.youtube.com/watch?v=NmM4pv8uQwI">Video: ICAPS 2014: Daniel Harabor on "Improving Jump Point Search"</a>
     *
     * References:
     * - Harabor, D. and Grastien, A., 2011. Online Graph Pruning for Pathfinding On Grid Maps. Proceedings of the AAAI Conference on Artificial Intelligence, 25 (1), 1114–1119.
     * - Harabor, D. and Grastien, A., 2012. The JPS Pathfinding System. Proceedings of the International Symposium on Combinatorial Search, 3 (1), 207–208.
     * - Harabor, D. and Grastien, A., 2014. Improving Jump Point Search. Proceedings International Conference on Automated Planning and Scheduling, ICAPS, 2014, 128–135.
     *
     * @note Guarantees the optimal path if the heuristic is admissible (never overestimates the cost), and the search
     *       space is a uniform-cost grid.
     *
     * @remarks Currently, only a 2D implementation is provided.
     *          Implementations for higher dimensions may be provided with future CHDR versions.
     *
     * @tparam params_t Type containing the search parameters.
     */
    template<typename params_t>
    struct [[maybe_unused]] jps final {

        friend class solver<jps, params_t>;

    private:

        using      index_t = typename params_t:: index_type;
        using     scalar_t = typename params_t::scalar_type;
        using     weight_t = typename params_t::weight_type;
        using      coord_t = typename params_t:: coord_type;
        using  direction_t = uint8_t;
        using   rotation_t = std::array<direction_t, 8U>;
        using     solver_t = solver<jps, params_t>;
        using  neighbour_t = typename std::remove_cv_t<std::remove_reference_t<decltype(params_t::maze)>>::neighbour_t;
        using neighbours_t = typename std::remove_cv_t<std::remove_reference_t<decltype(params_t::maze)>>::neighbours_t;

        static constexpr auto Kd = std::tuple_size_v<std::decay_t<typename params_t::coord_type>>;

        static_assert(std::is_arithmetic_v<scalar_t>, "scalar_t must be an integral or floating point type.");
        static_assert(std::is_integral_v<index_t>, "index_t must be an integral type.");

        struct node final : unmanaged_node<index_t> {

            scalar_t m_gScore;
            scalar_t m_fScore;

            direction_t m_direction;

            // ReSharper disable once CppPossiblyUninitializedMember
            [[nodiscard]] constexpr node() noexcept : unmanaged_node<index_t>() {} // NOLINT(*-use-equals-default)

            [[nodiscard]] constexpr node(index_t _index, const direction_t& _direction, scalar_t _gScore, scalar_t _hScore, const node* RESTRICT const _parent = nullptr) noexcept : unmanaged_node<index_t>(_index, _parent),
                m_gScore(_gScore),
                m_fScore(_gScore + _hScore),
                m_direction(_direction) {}

            ~node() = default;

            node           (const node&) = delete;
            node& operator=(const node&) = delete;

            [[nodiscard]] HOT constexpr node(node&& _other) noexcept = default;

#if __cplusplus > 202302L
            constexpr
#endif
            HOT node& operator=(node&& _other) noexcept = default;

            [[nodiscard]] HOT friend constexpr bool operator < (const node& _a, const node& _b) noexcept {
                return _a.m_fScore == _b.m_fScore ?
                       _a.m_gScore >  _b.m_gScore :
                       _a.m_fScore >  _b.m_fScore;
            }
        };

        static constexpr rotation_t s_identity { 0U, 1U, 2U,
                                                 3U,     4U,
                                                 5U, 6U, 7U };

        static constexpr rotation_t s_rotate_l { 2U, 4U, 7U,
                                                 1U,     6U,
                                                 0U, 3U, 5U };

        static constexpr rotation_t s_rotate_2 { 7U, 6U, 5U,
                                                 4U,     3U,
                                                 2U, 1U, 0U };

        static constexpr rotation_t s_rotate_r { 5U, 3U, 0U,
                                                 6U,     1U,
                                                 7U, 4U, 2U };

        static constexpr direction_t zero_direction_v { 3U };

        static constexpr std::array<rotation_t, 9U> s_lookup {
            /*  { -1, -1 }  :  0  */ s_rotate_2, // 0
            /*  {  0, -1 }  :  1  */ s_rotate_r, // 1
            /*  { -1,  0 }  :  2  */ s_rotate_2, // 2
            /*  {  0,  0 }  :  3  */ s_identity, // 3
            /*  {  1,  0 }  :  4  */ s_identity, // 4
            /*  {  0,  1 }  :  5  */ s_rotate_l, // 5
            /*  {  1,  1 }  :  6  */ s_identity, // 6
            /*  {  1, -1 }  :  2  */ s_rotate_r, // 7
            /*  { -1,  1 }  :  4  */ s_rotate_l, // 8
        };

        static constexpr bool is_straight(const direction_t& _direction) noexcept {
            return _direction == 1U ||
                   _direction == 2U ||
                   _direction == 4U ||
                   _direction == 5U;
        }

        [[nodiscard]] static constexpr auto get_direction(const coord_t& _from, const coord_t& _to) {

            coord_t dir;
            for (size_t i = 0U; i < Kd; ++i) {
                dir[i] = static_cast<index_t>(utils::sign<signed>(static_cast<signed>(_to[i]) - static_cast<signed>(_from[i])) + 1);
            }

            direction_t result{};

                 if (dir[0U] == 2U && dir[1U] == 0U) { result = 7U; }
            else if (dir[0U] == 0U && dir[1U] == 2U) { result = 8U; }
            else {
                result = static_cast<direction_t>(utils::to_1d(dir, coord_t { Kd }));
            }

            return result;
        }

        template <typename maze_t>
        [[nodiscard]] static constexpr neighbours_t go_find_jump_points(const maze_t& _maze, const coord_t& _current, const direction_t _direction, const coord_t& _end) {

            constexpr auto null_v = std::make_pair(false, coord_t{});

            const auto& neighbours = _maze.template get_neighbours<true>(_current);

            if (UNLIKELY(_direction == zero_direction_v)) { // START NODE:

                return {
                    neighbours[0U].first ? jump(_maze, neighbours[0U].second, _current, _end) : null_v, // FORCED
                    neighbours[1U].first ? jump(_maze, neighbours[1U].second, _current, _end) : null_v, // FORCED
                    neighbours[2U].first ? jump(_maze, neighbours[2U].second, _current, _end) : null_v, // FORCED
                    neighbours[3U].first ? jump(_maze, neighbours[3U].second, _current, _end) : null_v, // FORCED
                    neighbours[4U].first ? jump(_maze, neighbours[4U].second, _current, _end) : null_v, // FORCED
                    neighbours[5U].first ? jump(_maze, neighbours[5U].second, _current, _end) : null_v, // FORCED
                    neighbours[6U].first ? jump(_maze, neighbours[6U].second, _current, _end) : null_v, // FORCED
                    neighbours[7U].first ? jump(_maze, neighbours[7U].second, _current, _end) : null_v  // FORCED
                };
            }
            else {

                const auto& map = s_lookup[static_cast<size_t>(_direction)];

                if (is_straight(_direction)) { // STRAIGHT:

                    return {
                        neighbours[map[2U]].first && !neighbours[map[1U]].first ? jump(_maze, neighbours[map[2U]].second,   _current, _end) : null_v, // FORCED
                        neighbours[map[7U]].first && !neighbours[map[6U]].first ? jump(_maze, neighbours[map[7U]].second,   _current, _end) : null_v, // FORCED
                        neighbours[map[4U]].first                               ? jump(_maze, neighbours[map[4U]].second, _direction, _end) : null_v, // NATURAL
                        null_v,                                                                                                                       // NULL
                        null_v,                                                                                                                       // NULL
                        null_v,                                                                                                                       // NULL
                        null_v,                                                                                                                       // NULL
                        null_v                                                                                                                        // NULL
                    };
                }
                else if (neighbours[map[1U]].first || neighbours[map[3U]].first) { // DIAGONAL (if not blocked):

                    return {
                        neighbours[map[2U]].first && !neighbours[map[1U]].first ? jump(_maze, neighbours[map[2U]].second,   _current, _end) : null_v, // FORCED
                        neighbours[map[5U]].first && !neighbours[map[3U]].first ? jump(_maze, neighbours[map[5U]].second,   _current, _end) : null_v, // FORCED
                        neighbours[map[4U]].first                               ? jump(_maze, neighbours[map[4U]].second,   _current, _end) : null_v, // NATURAL
                        neighbours[map[6U]].first                               ? jump(_maze, neighbours[map[6U]].second,   _current, _end) : null_v, // NATURAL
                        neighbours[map[7U]].first                               ? jump(_maze, neighbours[map[7U]].second, _direction, _end) : null_v, // NATURAL
                        null_v,                                                                                                                       // NULL
                        null_v,                                                                                                                       // NULL
                        null_v                                                                                                                        // NULL
                    };
                }
                else {
                    return { null_v };
                }
            }
        }

        template <typename maze_t>
        [[nodiscard]] static constexpr neighbour_t jump(const maze_t& _maze, const coord_t& _current, const coord_t& _previous, const coord_t& _end) {
            return jump(_maze, _current, get_direction(_previous, _current), _end);
        }

        template <typename maze_t>
        [[nodiscard]] static constexpr neighbour_t jump(const maze_t& _maze, const coord_t& _current, const direction_t& _direction, const coord_t& _end) {

            if (UNLIKELY(_current == _end)) { // SOLUTION REACHED...
                return { true, _current };
            }
            else { // SEARCH FOR SOLUTION...

                const auto& neighbours = _maze.template get_neighbours<true>(_current);
                const auto& map = s_lookup[static_cast<size_t>(_direction)];

                const auto check_forced = [&neighbours, &map](size_t _a, size_t _b) ALWAYS_INLINE {
                    return neighbours[map[_a]].first && !neighbours[map[_b]].first;
                };

                if (is_straight(_direction)) { // STRAIGHT...

                    if (check_forced(2U, 1U) || check_forced(7U, 6U)) { // FORCED:
                        return { true, _current };
                    }
                    else if (neighbours[map[4U]].first) { // NATURAL:
                        return jump(_maze, neighbours[map[4U]].second, _direction, _end);
                    }
                }
                else if (neighbours[map[1U]].first || neighbours[map[3U]].first) { // DIAGONAL (if not blocked)...

                    if (check_forced(2U, 1U) || check_forced(5U, 3U)) { // FORCED:
                        return { true, _current };
                    }
                    else { // NATURAL:

                        for (const auto& i : { 4U, 6U }) {

                            if (neighbours[map[i]].first) {
                                if (jump(_maze, neighbours[map[i]].second, _current, _end).first) {
                                    return { true, _current };
                                }
                            }
                        }

                        if (neighbours[map[7U]].first) {
                            return jump(_maze, neighbours[map[7U]].second, _direction, _end);
                        }
                    }
                }
            }

            return { false, _current };
        }

        template <typename open_set_t, typename closed_set_t>
        [[nodiscard]] HOT static constexpr auto solve_internal(open_set_t& _open, closed_set_t& _closed, size_t _capacity, const params_t& _params) {

            static_assert(std::is_base_of_v<mazes::grid<coord_t, weight_t>, std::remove_cv_t<std::remove_reference_t<decltype(_params.maze)>>>,
                          "JPS only supports grid mazes.");

            if constexpr (Kd == 2U) {

                const auto s = utils::to_1d(_params.start, _params.size);
                const auto e = utils::to_1d(_params.end  , _params.size);

                  _open.emplace_nosort(s, zero_direction_v, static_cast<scalar_t>(0), _params.h(_params.start, _params.end) * _params.weight);
                _closed.emplace(s);

                // Main loop:
                while (LIKELY(!_open.empty())) {

                    auto curr(std::move(_open.top()));
                    _open.pop();

                    if (curr.m_index != e) { // SEARCH FOR SOLUTION...

                        node* RESTRICT curr_ptr(nullptr);

                        const auto coord = utils::to_nd(curr.m_index, _params.size);

                        for (const auto& n_data : go_find_jump_points(_params.maze, coord, curr.m_direction, _params.end)) {

                            if (const auto& [nActive, nCoord] = n_data; nActive) {

                                const auto n = utils::to_1d(nCoord, _params.size);

                                constexpr scalar_t nDistance{1};

                                if (!_closed.contains(n)) {
                                    solver_t::solver_utils::preallocate_emplace(_closed, n, _capacity, _params.maze.count());

                                    if (curr_ptr == nullptr) {
                                        curr_ptr = new (_params.monotonic_pmr->allocate(sizeof(node), alignof(node))) node(std::move(curr));
                                    }

                                    if constexpr (params_t::lazy_sorting::value) {
                                        _open.emplace_nosort(n, get_direction(coord, nCoord), curr_ptr->m_gScore + nDistance, _params.h(nCoord, _params.end) * _params.weight, curr_ptr);
                                    }
                                    else {
                                        _open.emplace(n, get_direction(coord, nCoord), curr_ptr->m_gScore + nDistance, _params.h(nCoord, _params.end) * _params.weight, curr_ptr);
                                    }
                                }
                            }
                        }
                    }
                    else { // SOLUTION REACHED ...

                        if constexpr (std::is_same_v<std::decay_t<decltype(_open)>, heap<node>>) {
                            _open.wipe();
                        }
                        else {
                            _open = {};
                        }
                        _closed = {};

                        return solver_t::solver_utils::rbacktrack(curr, _params.size, curr.m_gScore);
                    }
                }
            }

            return std::vector<coord_t>{};
        }

        [[maybe_unused, nodiscard]] static auto invoke(const params_t& _params) {

            const auto capacity = solver_t::solver_utils::determine_capacity(_params);

            existence_set closed(_params.monotonic_pmr);
            closed.reserve(capacity);

            heap<node> open(_params.polytonic_pmr);
            try {
                open.reserve(capacity / 8U);
            }
            catch (...) {} // NOLINT(*-empty-catch)

            return solve_internal(open, closed, capacity, _params);
        }
    };

} //chdr::solvers

#endif //CHDR_JPS_HPP