/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_GJPS_HPP
#define CHDR_GJPS_HPP

#include <cstddef>
#include <type_traits>
#include <utility>

#include "base/solver.hpp"
#include "mazes/grid.hpp"
#include "types/append_only_allocator.hpp"
#include "types/coord.hpp"
#include "types/existence_set.hpp"
#include "utils/intrinsics.hpp"
#include "utils/utils.hpp"

namespace chdr::solvers {

    template<size_t Kd, typename params_t>
    struct [[maybe_unused]] gjps final {

        friend class solver<gjps, Kd, params_t>;

    private:

        using     index_t = typename params_t:: index_type;
        using    scalar_t = typename params_t::scalar_type;
        using    weight_t = typename params_t::weight_type;
        using     coord_t = typename params_t:: coord_type;
        using direction_t = char;
        using  rotation_t = std::array<direction_t, 8U>;
        using    solver_t = solver<gjps, Kd, params_t>;

        static_assert(std::is_arithmetic_v<scalar_t>, "scalar_t must be an integral or floating point type.");
        static_assert(std::is_integral_v<index_t>, "index_t must be an integral type.");

        struct node final : managed_node<index_t, node> {

            scalar_t m_gScore;
            scalar_t m_fScore;

            direction_t m_direction;

            // ReSharper disable once CppPossiblyUninitializedMember
            [[nodiscard]] constexpr node() noexcept : managed_node<index_t, node>() {} // NOLINT(*-use-equals-default)

            [[nodiscard]] constexpr node(const index_t& _index, const direction_t& _direction, const scalar_t& _gScore, const scalar_t& _hScore, node* RESTRICT const _parent = nullptr) noexcept : managed_node<index_t, node>(_index, _parent),
                m_gScore(_gScore),
                m_fScore(_gScore + _hScore),
                m_direction(_direction) {}

            [[nodiscard]] friend constexpr bool operator < (const node& _a, const node& _b) noexcept {
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
                dir[i] = static_cast<index_t>(utils::sign<signed>(static_cast<signed>(_to[i]) - static_cast<signed>(_from[i])) + 1U);
            }

            direction_t result{};

                 if (dir[0U] == 2U && dir[1U] == 0U) { result = 7U; }
            else if (dir[0U] == 0U && dir[1U] == 2U) { result = 8U; }
            else {
                result = utils::to_1d<index_t>(dir, { Kd });
            }

            return result;
        }

        [[nodiscard]] static constexpr std::array<std::pair<bool, coord_t>, 8U> go_find_jump_points(const mazes::grid<Kd, weight_t>& _maze, const coord_t& _current, const direction_t _direction, const coord_t& _end) {

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

                const auto& map = s_lookup[_direction];

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

        [[nodiscard]] static constexpr auto jump(const mazes::grid<Kd, weight_t>& _maze, const coord_t& _current, const coord_t& _previous, const coord_t& _end) {
            return jump(_maze, _current, get_direction(_previous, _current), _end);
        }

        [[nodiscard]] static constexpr std::pair<bool, coord_t> jump(const mazes::grid<Kd, weight_t>& _maze, const coord_t& _current, const direction_t& _direction, const coord_t& _end) {

            if (UNLIKELY(_current == _end)) { // SOLUTION REACHED...
                return { true, _current };
            }
            else { // SEARCH FOR SOLUTION...

                const auto& neighbours = _maze.template get_neighbours<true>(_current);
                const auto& map = s_lookup[_direction];

                const auto check_forced = [&neighbours, &map](const size_t& _a, const size_t& _b) ALWAYS_INLINE {
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
        [[nodiscard]] static constexpr auto solve_internal(open_set_t& _open, closed_set_t& _closed, const size_t& _capacity, const params_t& _params) {
            //
            // static_assert(std::is_base_of_v<mazes::grid<Kd, weight_t>, std::remove_cv_t<std::remove_reference_t<decltype(_params.maze)>>>,
            //               "JPS only supports mazes of type grid<Kd, weight_t>.");

            if constexpr (Kd == 2U) {

                const auto s = utils::to_1d(_params.start, _params.size);
                const auto e = utils::to_1d(_params.end  , _params.size);

                  _open.emplace_nosort(s, zero_direction_v, static_cast<scalar_t>(0), _params.h(_params.start, _params.end) * _params.weight);
                _closed.emplace(s);

                // Main loop:
                while (!_open.empty()) {

                    auto curr(std::move(_open.top()));
                    _open.pop();

                    if (curr.m_index != e) { // SEARCH FOR SOLUTION...

                        node* RESTRICT curr_ptr = nullptr;

                        const auto coord = utils::to_nd(curr.m_index, _params.size);

                        for (const auto& n_data : go_find_jump_points(_params.maze, coord, curr.m_direction, _params.end)) {

                            if (const auto& [nActive, nCoord] = n_data; nActive) {

                                const auto n = utils::to_1d(nCoord, _params.size);

                                constexpr scalar_t nDistance{1};

                                if (!_closed.contains(n)) {
                                    utils::preallocate_emplace(_closed, n, _capacity, _params.maze.count());

                                    if (curr_ptr == nullptr) {
                                        node::alloc.construct(curr_ptr = node::alloc.allocate(1U), std::move(curr)); // Note: 'current' is now moved!
                                    }

                                    _open.emplace_nosort(n, get_direction(coord, nCoord), curr_ptr->m_gScore + nDistance, _params.h(nCoord, _params.end) * _params.weight, curr_ptr);
                                }
                            }
                        }

                        if (curr_ptr != nullptr) {
                            _open.reheapify(_open.back());
                        }
                    }
                    else { // SOLUTION REACHED ...

                        _open   = {};
                        _closed = {};

                        const auto result = utils::rbacktrack(curr, _params.size, curr.m_gScore);

                        node::alloc.reset();

                        return result;
                    }
                }
            }

            _open   = {};
            _closed = {};
            node::alloc.reset();

            return std::vector<coord_t>{};
        }

        [[maybe_unused, nodiscard]] static auto execute(const params_t& _params) {

            const auto capacity = solver_t::determine_capacity(_params);

            existence_set closed;
            closed.reserve(capacity);

            heap<node> open;
            try {
                open.reserve(capacity / 8U);
            }
            catch ([[maybe_unused]] const std::exception& e) {} // NOLINT(*-empty-catch)

            return solve_internal(open, closed, capacity, _params);
        }
    };

} //chdr::solvers

#endif //CHDR_GJPS_HPP