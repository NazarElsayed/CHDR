/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_JPS_HPP
#define CHDR_JPS_HPP

#include <cstddef>
#include <cstdint>
#include <map>
#include <type_traits>
#include <utility>

#include "base/solver.hpp"
#include "mazes/grid.hpp"
#include "types/coord.hpp"
#include "types/existence_set.hpp"
#include "types/stable_forward_buf.hpp"
#include "utils/intrinsics.hpp"
#include "utils/utils.hpp"

namespace chdr::solvers {

    template<size_t Kd, typename scalar_t, typename index_t, typename params_t>
    struct [[maybe_unused]] jps final {

        friend struct solver<jps, Kd, scalar_t, index_t, params_t>;

        static_assert(std::is_arithmetic_v<scalar_t>, "scalar_t must be an integral or floating point type.");
        static_assert(std::is_integral_v<index_t>, "index_t must be an integral type.");

    private:

        using    solver_t = solver<jps, Kd, scalar_t, index_t, params_t>;
        using     coord_t = coord<index_t, Kd>;
        using    weight_t = typename params_t::weight_type;
        using direction_t = coord<int8_t, Kd>;
        using  rotation_t = std::array<uint8_t, 8U>;

        struct node final : unmanaged_node<index_t> {

            direction_t m_direction;

            scalar_t m_gScore;
            scalar_t m_fScore;

            // ReSharper disable once CppPossiblyUninitializedMember
            [[nodiscard]] constexpr node() noexcept : unmanaged_node<index_t>() {}

            [[nodiscard]] constexpr node(const index_t& _index, const direction_t& _direction, const scalar_t& _gScore, const scalar_t& _hScore, const node* RESTRICT const _parent = nullptr) noexcept : unmanaged_node<index_t>(_index, _parent),
                m_direction(_direction),
                m_gScore(_gScore),
                m_fScore(_gScore + _hScore) {}

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

        static constexpr std::array<rotation_t, 9U> s_rotation_lookup {
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

        static constexpr auto increment(const direction_t& _value) noexcept {

            std::array<index_t, Kd> result;

            for (size_t i = 0U; i < _value.size(); ++i) {
                result[i] = _value[i] + 1U;
            }

            return result;
        }

        static constexpr auto get_rotation(const direction_t& _direction) noexcept {

            index_t index;

            if (_direction[0U] != 1 || _direction[1U] != -1) {
                if (_direction[0U] != -1 || _direction[1U] != 1) {
                    index = utils::to_1d<index_t>(increment(_direction), utils::build_array<index_t, Kd, 2U>());
                }
                else { index = 8U; }
            }
            else { index = 7U; }

            return s_rotation_lookup[index];
        }

        [[nodiscard]] static constexpr auto get_direction(const coord_t& _from, const coord_t& _to) {

            direction_t result;

            for (size_t i = 0U; i < Kd; ++i) {
                result[i] = static_cast<int8_t>(utils::sign<int8_t>(static_cast<signed>(_to[i]) - static_cast<signed>(_from[i])));
            }

            return result;
        }

        [[nodiscard]] static constexpr std::vector<coord_t> find_jump_points(const mazes::grid<Kd, weight_t>& _maze, const coord_t& _current, const direction_t _direction, const coord_t& _end) {

            std::vector<coord_t> result;
            result.reserve(8U);

            const auto neighbours = _maze.template get_neighbours<true>(_current);

            // Start Node:
            if (_direction[0U] == 0 && _direction[1U] == 0) {
                for (auto& neighbour : neighbours) {
                    if (neighbour.first) {
                        if (const auto& [nActive, nCoord] = jump(_maze, neighbour.second, _current, _end); nActive) {
                            result.emplace_back(nCoord);
                        }
                    }
                }
            }
            else {

                const auto map = get_rotation(_direction);

                if (_direction[0U] == 0 || _direction[1U] == 0) { // Straight Direction:

                    // Check and expand forced neighbours:
                    if (neighbours[map[2U]].first && !neighbours[map[1U]].first) {
                        if (const auto& [nActive, nCoord] = jump(_maze, neighbours[map[2U]].second, _current, _end); nActive) {
                            result.emplace_back(nCoord);
                        }
                    }
                    if (neighbours[map[7U]].first && !neighbours[map[6U]].first) {
                        if (const auto& [nActive, nCoord] = jump(_maze, neighbours[map[7U]].second, _current, _end); nActive) {
                            result.emplace_back(nCoord);
                        }
                    }

                    // Expand natural neighbours:
                    if (neighbours[map[4U]].first) {
                        if (const auto& [nActive, nCoord] = jump(_maze, neighbours[map[4U]].second, _direction, _end); nActive) {
                            result.emplace_back(nCoord);
                        }
                    }
                }
                else if (neighbours[map[1U]].first || neighbours[map[3U]].first) { // Diagonal Direction (if diagonal is not blocked)

                    // Check and expand forced neighbours:
                    if (neighbours[map[2U]].first && !neighbours[map[1U]].first) {
                        if (const auto& [nActive, nCoord] = jump(_maze, neighbours[map[2U]].second, _current, _end); nActive) {
                            result.emplace_back(nCoord);
                        }
                    }
                    if (neighbours[map[5U]].first && !neighbours[map[3U]].first) {
                        if (const auto& [nActive, nCoord] = jump(_maze, neighbours[map[5U]].second, _current, _end); nActive) {
                            result.emplace_back(nCoord);
                        }
                    }

                    // Expand natural neighbours:
                    if (neighbours[map[4U]].first) {
                        if (const auto& [nActive, nCoord] = jump(_maze, neighbours[map[4U]].second, _current, _end); nActive) {
                            result.emplace_back(nCoord);
                        }
                    }
                    if (neighbours[map[6U]].first) {
                        if (const auto& [nActive, nCoord] = jump(_maze, neighbours[map[6U]].second, _current, _end); nActive) {
                            result.emplace_back(nCoord);
                        }
                    }
                    if (neighbours[map[7U]].first) {
                        if (const auto& [nActive, nCoord] = jump(_maze, neighbours[map[7U]].second, _direction, _end); nActive) {
                            result.emplace_back(nCoord);
                        }
                    }
                }
            }

            return result;
        }

        [[nodiscard]] static constexpr std::pair<bool, coord_t> jump(const mazes::grid<Kd, weight_t>& _maze, const coord_t& _current, const coord_t& _previous, const coord_t& _end) {
            return jump(_maze, _current, get_direction(_previous, _current), _end);
        }

        [[nodiscard]] static constexpr std::pair<bool, coord_t> jump(const mazes::grid<Kd, weight_t>& _maze, const coord_t& _current, const direction_t& _direction, const coord_t& _end) {

            std::pair<bool, coord_t> result;
            result.second = _current;

            if (_current == _end) {
                result.first = true;
            }
            else {

                const auto neighbours = _maze.template get_neighbours<true>(_current);

                const auto map = get_rotation(_direction);

                if (_direction[0U] == 0 || _direction[1U] == 0) { // Straight Direction

                    // Check for forced neighbours:
                    if ((neighbours[map[2U]].first && !neighbours[map[1U]].first) || (neighbours[map[7U]].first && !neighbours[map[6U]].first)) {
                        result.first = true;
                    }
                    else if (neighbours[map[4U]].first) { // Expand natural neighbours:
                        result = jump(_maze, neighbours[map[4U]].second, _direction, _end);
                    }
                }
                else if (neighbours[map[1U]].first || neighbours[map[3U]].first) { // Diagonal Direction (if diagonal is not blocked)

                    // Check for forced neighbours:
                    if ((neighbours[map[2U]].first && !neighbours[map[1U]].first) || (neighbours[map[5U]].first && !neighbours[map[3U]].first)) {
                        result.first = true;
                    }
                    else { // Expand natural neighbours:

                        if (neighbours[map[4U]].first) {
                            result.first = jump(_maze, neighbours[map[4U]].second, _current, _end).first;
                        }
                        if (!result.first && neighbours[map[6U]].first) {
                            result.first = jump(_maze, neighbours[map[6U]].second, _current, _end).first;
                        }
                        if (!result.first && neighbours[map[7U]].first) {
                            result = jump(_maze, neighbours[map[7U]].second, _direction, _end);
                        }
                    }
                }
            }

            return result;
        }

        template <typename open_set_t, typename closed_set_t, typename buf_t>
        [[nodiscard]] static constexpr auto solve_internal(open_set_t& _open, closed_set_t& _closed, buf_t& _buf, const size_t& _capacity, const params_t& _params) {

            static_assert(std::is_base_of_v<mazes::grid<Kd, weight_t>, std::remove_cv_t<std::remove_reference_t<decltype(_params.maze)>>>,
                          "JPS only supports mazes of type grid<Kd, weight_t>.");

            const auto s = utils::to_1d(_params.start, _params.size);
            const auto e = utils::to_1d(_params.end  , _params.size);

            _open.emplace(s, utils::build_array<int8_t, Kd, 0U>(), static_cast<scalar_t>(0), _params.h(_params.start, _params.end));

            _closed.allocate(s, _capacity, _params.maze.count());
            _closed.emplace (s);

            // Main loop:
            while (!_open.empty()) {

                auto curr(std::move(_open.top()));
                _open.pop();

                if (curr.m_index != e) { // SEARCH FOR SOLUTION...

                    node* RESTRICT curr_ptr = nullptr;

                    const auto coord = utils::to_nd(curr.m_index, _params.size);

                    for (const auto& successor : find_jump_points(_params.maze, coord, curr.m_direction, _params.end)) {

                        const auto n = utils::to_1d(successor, _params.size);

                        constexpr scalar_t nDistance{1};

                        if (!_closed.contains(n)) {
                             _closed.allocate(n, _capacity, _params.maze.count());
                             _closed.emplace (n);

                            if (curr_ptr == nullptr) {
                                curr_ptr = &_buf.emplace(std::move(curr)); // Note: 'current' is now moved!
                            }

                            _open.emplace(n, get_direction(coord, successor), curr.m_gScore + nDistance, _params.h(successor, _params.end) * _params.weight, curr_ptr);
                        }
                    }
                }
                else { // SOLUTION REACHED ...
                    return curr.template backtrack<node>(_params.size, curr.m_gScore);
                }
            }

            return std::vector<coord_t>{};
        }

        [[maybe_unused, nodiscard]] static constexpr auto execute(const params_t& _params) {

            const auto s = utils::to_1d(_params.start, _params.size);

            const auto capacity = solver_t::determine_capacity(_params);

            existence_set<low_memory_usage> closed({ s }, capacity);

            heap<node> open;
            open.reserve(capacity / 8U);

            stable_forward_buf<node> buf;

            return solve_internal(open, closed, buf, capacity, _params);
        }
    };

} //chdr::solvers

#endif //CHDR_JPS_HPP