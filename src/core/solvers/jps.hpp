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

#include "base/bsolver.hpp"
#include "mazes/grid.hpp"
#include "types/existence_set.hpp"
#include "types/stable_forward_buf.hpp"
#include "utils/utils.hpp"

namespace chdr::solvers {

    template<typename weight_t, const size_t Kd, typename scalar_t, typename index_t, typename params_t>
    class [[maybe_unused]] jps final : public bsolver<weight_t, Kd, scalar_t, index_t, params_t> {

        static_assert(std::is_integral_v<scalar_t> || std::is_floating_point_v<scalar_t>, "scalar_t must be either an integral or floating point type");
        static_assert(std::is_integral_v<index_t>, "index_t must be an integral type.");

    private:

        using coord_t = coord<index_t, Kd>;

        static constexpr std::array<uint8_t, 8U> s_rotate_l { 2U, 4U, 7U,
                                                              1U,     6U,
                                                              0U, 3U, 5U };

        static constexpr std::array<uint8_t, 8U> s_rotate_2 { 7U, 6U, 5U,
                                                              4U,     3U,
                                                              2U, 1U, 0U };

        static constexpr std::array<uint8_t, 8U> s_rotate_r { 5U, 3U, 0U,
                                                              6U,     1U,
                                                              7U, 4U, 2U };

        const std::map<std::array<int8_t, 2U>, std::array<uint8_t, 8U>> m_rotation_map {
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

        struct jps_node final : unmanaged_node<index_t> {

            std::array<int8_t, 2U> m_direction;

            scalar_t m_gScore;
            scalar_t m_fScore;

            // ReSharper disable once CppPossiblyUninitializedMember
            [[nodiscard]] constexpr jps_node() noexcept : unmanaged_node<index_t>() {}

            [[nodiscard]] constexpr jps_node(const index_t& _index, const std::array<int8_t, 2U>& _direction, const scalar_t& _gScore, const scalar_t& _hScore, const jps_node* RESTRICT const _parent) noexcept : unmanaged_node<index_t>(_index, _parent),
                m_direction(_direction),
                m_gScore(_gScore),
                m_fScore(_gScore + _hScore) {}

            struct max {

                [[nodiscard]] constexpr bool operator () (const jps_node& _a, const jps_node& _b) const noexcept {

                    return _a.m_fScore == _b.m_fScore ?
                           _a.m_gScore >  _b.m_gScore :
                           _a.m_fScore >  _b.m_fScore;
                }
            };
        };

        template <typename T>
        static constexpr int sign(const T _val) {
            return (static_cast<T>(0) < _val) - (_val < static_cast<T>(0));
        }

        std::vector<coord_t> find_jump_points(const mazes::grid<Kd, weight_t>& _maze, const coord_t& _current, const std::array<int8_t, 2U> _direction, const coord_t& _end) const {

            std::vector<coord_t> result;

            const auto neighbours = _maze. template get_neighbours<true>(_current);
            const auto map = m_rotation_map.at(_direction);

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

        [[nodiscard]]
        std::pair<bool, coord_t> jump(const mazes::grid<Kd, weight_t>& _maze, const coord_t& _current, const coord_t& _previous, const coord_t& _end) const {

            const std::array<int8_t, 2U> direction { static_cast<int8_t>(sign(static_cast<int>(_current[0U]) - static_cast<int>(_previous[0U]))) ,
                                                     static_cast<int8_t>(sign(static_cast<int>(_current[1U]) - static_cast<int>(_previous[1U]))) };

            return jump(_maze, _current, direction, _end);
        }

        [[nodiscard]]
        std::pair<bool, coord_t> jump(const mazes::grid<Kd, weight_t>& _maze, const coord_t& _current, const std::array<int8_t, 2U>& _direction, const coord_t& _end) const {

            std::pair<bool, coord_t> result { false, _current };

            const auto neighbours = _maze. template get_neighbours<true>(_current);
            const auto map = m_rotation_map.at(_direction);

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

        [[maybe_unused, nodiscard]] std::vector<coord_t> execute(const params_t& _params) const override {

            std::vector<coord_t> result;

            const auto s = utils::to_1d(_params._start, _params._maze.size());
            const auto e = utils::to_1d(_params._end  , _params._maze.size());

            // Create closed set:
            const auto capacity = std::max(_params._capacity, std::max(s, e));
            existence_set<low_memory_usage> closed({ s }, capacity);

            // Create open set:
            heap<jps_node, typename jps_node::max> open(capacity / 8U);
            open.emplace(s, std::array<int8_t, 2U>{ 0, 0 }, static_cast<scalar_t>(0), _params._h(_params._start, _params._end), nullptr);

            // Create buffer:
            stable_forward_buf<jps_node> buf;

            // Main loop:
            while (!open.empty()) {

                auto curr(std::move(open.top()));
                open.pop();

                if (curr.m_index != e) { // SEARCH FOR SOLUTION...

                    closed.allocate(curr.m_index, capacity, _params._maze.count());
                    closed.emplace(curr.m_index);

                    auto      coord = utils::to_nd(curr.m_index, _params._maze.size());
                    auto successors = find_jump_points(_params._maze, coord, curr.m_direction, _params._end);

                    for (const auto& successor : successors) {

                        const auto n = utils::to_1d(successor, _params._maze.size());

                        constexpr auto nDistance = static_cast<scalar_t>(1);

                        if (!closed.contains(n)) {
                             closed.allocate(n, capacity, _params._maze.count());
                             closed.emplace(n);

                            const std::array<int8_t, 2U> direction { static_cast<int8_t>(sign(static_cast<int>(successor[0U]) - static_cast<int>(coord[0U]))) ,
                                                                     static_cast<int8_t>(sign(static_cast<int>(successor[1U]) - static_cast<int>(coord[1U]))) };

                            // Create a parent node and transfer ownership of 'current' to it. Note: 'current' is now moved!
                            open.emplace(n, direction, curr.m_gScore + static_cast<scalar_t>(nDistance), _params._h(successor, _params._end) * _params._weight, &buf.emplace(std::move(curr)));
                        }
                    }
                }
                else { // SOLUTION REACHED ...

                    result = curr.template backtrack<jps_node>(_params._maze.size(), curr.m_gScore);

                    break;
                }
            }

            return result;
        }

    };

} //chdr::solvers

#endif //CHDR_JPS_HPP