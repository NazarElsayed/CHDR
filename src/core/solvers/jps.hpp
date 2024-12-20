/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_JPS_HPP
#define CHDR_JPS_HPP

#include <map>

#include "base/solver.hpp"
#include "mazes/grid.hpp"
#include "types/existence_set.hpp"
#include "types/stable_forward_buf.hpp"
#include "utils/intrinsics.hpp"
#include "utils/utils.hpp"

namespace chdr::solvers {

    template<size_t Kd, typename scalar_t, typename index_t, typename params_t>
    struct [[maybe_unused]] jps final {

        friend struct solver<jps, Kd, scalar_t, index_t, params_t>;

        static_assert(std::is_integral_v<scalar_t> || std::is_floating_point_v<scalar_t>, "scalar_t must be either an integral or floating point type");
        static_assert(std::is_integral_v<index_t>, "index_t must be an integral type.");

    private:

        using solver_t = solver<jps, Kd, scalar_t, index_t, params_t>;
        using  coord_t = coord<index_t, Kd>;
        using weight_t = typename params_t::weight_type;

        static constexpr std::array<uint8_t, 8U> s_rotate_l { 2U, 4U, 7U,
                                                              1U,     6U,
                                                              0U, 3U, 5U };

        static constexpr std::array<uint8_t, 8U> s_rotate_2 { 7U, 6U, 5U,
                                                              4U,     3U,
                                                              2U, 1U, 0U };

        static constexpr std::array<uint8_t, 8U> s_rotate_r { 5U, 3U, 0U,
                                                              6U,     1U,
                                                              7U, 4U, 2U };

        inline static const std::map<std::array<int8_t, 2U>, std::array<uint8_t, 8U>> s_rotation_map {
                { { 0,  0}, { 0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U } },
                { { 1,  0}, { 0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U } },
                { { 1,  1}, { 0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U } },
                { { 0,  1}, s_rotate_l },
                { {-1,  1}, s_rotate_l },
                { {-1,  0}, s_rotate_2 },
                { {-1, -1}, s_rotate_2 },
                { { 0, -1}, s_rotate_r },
                { { 1, -1}, s_rotate_r }
        };

        struct node final : unmanaged_node<index_t> {

            std::array<int8_t, 2U> m_direction;

            scalar_t m_gScore;
            scalar_t m_fScore;

            // ReSharper disable once CppPossiblyUninitializedMember
            [[nodiscard]] constexpr node() noexcept : unmanaged_node<index_t>() {}

            [[nodiscard]] constexpr node(const index_t& _index, const std::array<int8_t, 2U>& _direction, const scalar_t& _gScore, const scalar_t& _hScore, const node* RESTRICT const _parent = nullptr) noexcept : unmanaged_node<index_t>(_index, _parent),
                m_direction(_direction),
                m_gScore(_gScore),
                m_fScore(_gScore + _hScore) {}

            [[nodiscard]] friend constexpr bool operator < (const node& _a, const node& _b) noexcept {
                return _a.m_fScore == _b.m_fScore ?
                       _a.m_gScore >  _b.m_gScore :
                       _a.m_fScore >  _b.m_fScore;
            }
        };

        [[nodiscard]] static constexpr std::vector<coord_t> find_jump_points(const mazes::grid<Kd, weight_t>& _maze, const coord_t& _current, const std::array<int8_t, 2U> _direction, const coord_t& _end) {

            std::vector<coord_t> result;

            const auto neighbours = _maze. template get_neighbours<true>(_current);
            const auto map = s_rotation_map.at(_direction);

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
            // Straight Direction:
            else if (_direction[0U] == 0 || _direction[1U] == 0) {

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
            // Diagonal Direction:
            else {

                // Check diagonal is not blocked:
                if (neighbours[map[1U]].first ||
                    neighbours[map[3U]].first) {

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

            const std::array<int8_t, 2U> direction { static_cast<int8_t>(utils::sign<int8_t>(static_cast<signed>(_current[0U]) - static_cast<signed>(_previous[0U]))) ,
                                                     static_cast<int8_t>(utils::sign<int8_t>(static_cast<signed>(_current[1U]) - static_cast<signed>(_previous[1U]))) };

            return jump(_maze, _current, direction, _end);
        }

        [[nodiscard]] static constexpr std::pair<bool, coord_t> jump(const mazes::grid<Kd, weight_t>& _maze, const coord_t& _current, const std::array<int8_t, 2U>& _direction, const coord_t& _end) {

            std::pair<bool, coord_t> result { false, _current };

            const auto neighbours = _maze. template get_neighbours<true>(_current);
            const auto map = s_rotation_map.at(_direction);

            if (_direction[0U] == 0 || _direction[1U] == 0) { // Straight Direction

                if (_current == _end) {
                    result.first = true;
                }
                else {

                    // Check for forced neighbours:
                    if ((neighbours[map[2U]].first && !neighbours[map[1U]].first) ||
                        (neighbours[map[7U]].first && !neighbours[map[6U]].first)) {

                        result.first = true;
                    }
                    // Expand natural neighbours:
                    else {
                        if (neighbours[map[4U]].first) {
                            result = jump(_maze, neighbours[map[4U]].second, _direction, _end);
                        }
                    }
                }
            }
            else { // Diagonal Direction

                // Check diagonal is not blocked:
                if (neighbours[map[1U]].first ||
                    neighbours[map[3U]].first) {

                    if (_current == _end) {
                        result.first = true;
                    }
                    else {

                        // Check for forced neighbours:
                        if ((neighbours[map[2U]].first && !neighbours[map[1U]].first) ||
                            (neighbours[map[5U]].first && !neighbours[map[3U]].first)) {

                            result.first = true;
                        }
                        // Expand natural neighbours:
                        else {
                            if (neighbours[map[4U]].first) {
                                result.first = jump(_maze, neighbours[map[4U]].second, _current, _end).first;
                            }

                            if (!result.first) {
                                if (neighbours[map[6U]].first) {
                                    result.first = jump(_maze, neighbours[map[6U]].second, _current, _end).first;
                                }
                            }

                            if (!result.first) {
                                if (neighbours[map[7U]].first) {
                                    result = jump(_maze, neighbours[map[7U]].second, _direction, _end);
                                }
                            }
                        }
                    }
                }
            }

            return result;
        }

        template <typename open_set_t, typename closed_set_t, typename buf_t>
        [[nodiscard]] static constexpr auto solve_internal(open_set_t& _open, closed_set_t& _closed, buf_t& _buf, const size_t& _capacity, const params_t& _params) {

            static_assert(std::is_same_v<decltype(_params.maze), mazes::grid<Kd, weight_t>>, "JPS only supports mazes of type grid<Kd, weight_t>.");
            
            const auto s = utils::to_1d(_params.start, _params.size);
            const auto e = utils::to_1d(_params.end  , _params.size);

            _open.emplace(s, std::array<int8_t, 2U> {{ 0, 0 }}, static_cast<scalar_t>(0), _params.h(_params.start, _params.end));

            // Main loop:
            while (!_open.empty()) {

                auto curr(std::move(_open.top()));
                _open.pop();

                if (curr.m_index != e) { // SEARCH FOR SOLUTION...

                    _closed.allocate(curr.m_index, _capacity, _params.maze.count());
                    _closed.emplace (curr.m_index);

                    node* RESTRICT curr_ptr = nullptr;

                    const auto coord = utils::to_nd(curr.m_index, _params.size);

                    for (const auto& successor : find_jump_points(_params.maze, coord, curr.m_direction, _params.end)) {

                        const auto n = utils::to_1d(successor, _params.size);

                        constexpr scalar_t nDistance{1};

                        if (!_closed.contains(n)) {
                             _closed.allocate(n, _capacity, _params.maze.count());
                             _closed.emplace (n);

                            const std::array<int8_t, 2U> direction { static_cast<int8_t>(utils::sign<int8_t>(static_cast<signed>(successor[0U]) - static_cast<signed>(coord[0U]))) ,
                                                                     static_cast<int8_t>(utils::sign<int8_t>(static_cast<signed>(successor[1U]) - static_cast<signed>(coord[1U]))) };

                            if (curr_ptr == nullptr) {
                                curr_ptr = &_buf.emplace(std::move(curr)); // Note: 'current' is now moved!
                            }

                            _open.emplace(n, direction, curr.m_gScore + nDistance, _params.h(successor, _params.end) * _params.weight, curr_ptr);
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