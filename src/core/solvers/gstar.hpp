/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_GSTAR_HPP
#define CHDR_GSTAR_HPP

#include <memory>

#include "base/bsolver.hpp"
#include "mazes/graph.hpp"
#include "mazes/grid.hpp"
#include "mazes/base/imaze.hpp"
#include "types/existence_set.hpp"
#include "types/heap.hpp"
#include "utils/utils.hpp"

namespace chdr::solvers {

    template<typename weight_t, const size_t Kd, typename scalar_t, typename index_t>
    class [[maybe_unused]] gstar final : public bsolver<weight_t, Kd, scalar_t, index_t> {

        static_assert(std::is_integral_v<scalar_t> || std::is_floating_point_v<scalar_t>, "scalar_t must be either an integral or floating point type");
        static_assert(std::is_integral_v<index_t>, "index_t must be an integral type.");

    private:

        using coord_t = coord<index_t, Kd>;

        struct gs_node final : managed_node<index_t> {

            scalar_t m_gScore;
            scalar_t m_fScore;

            [[nodiscard]] constexpr gs_node() : managed_node<index_t>() {}

            [[nodiscard]] constexpr gs_node(const index_t& _index, const scalar_t& _gScore, const scalar_t& _hScore) : managed_node<index_t>(_index),
                m_gScore(_gScore),
                m_fScore(_gScore + _hScore) {}

            [[nodiscard]] constexpr gs_node(const index_t& _index, const scalar_t& _gScore, const scalar_t& _hScore, gs_node&& _parent) : managed_node<index_t>(_index, std::move(_parent)),
                m_gScore(_gScore),
                m_fScore(_gScore + _hScore) {}

            struct max {

                [[nodiscard]] constexpr bool operator () (const gs_node& _a, const gs_node& _b) const {

                    return _a.m_fScore == _b.m_fScore ?
                        _a.m_gScore > _b.m_gScore :
                        _a.m_fScore > _b.m_fScore;
                }
            };
        };

    public:

        [[maybe_unused]]
        std::vector<coord_t> execute(const mazes::graph<index_t, scalar_t>& _maze, const coord_t& _start, const coord_t& _end, const coord_t& _size, scalar_t (*_h)(const coord_t&, const coord_t&), const scalar_t& _weight, size_t _capacity) const override {

            std::vector<coord_t> result;

            const auto s = utils::to_1d(_start, _size);
            const auto e = utils::to_1d(_end,   _size);

            // Create closed set:
            _capacity = std::max(_capacity, std::max(s, e));
            existence_set closed({ s }, _capacity);

            // Create open set:
            heap<gs_node, typename gs_node::max> open;
            open.emplace(s, static_cast<scalar_t>(0), _h(_start, _end));

            // Main loop:
            while (!open.empty()) {

                auto curr(std::move(open.top()));
                open.pop();

                if (curr.m_index != e) { // SEARCH FOR SOLUTION...

                    closed.allocate(curr.m_index, _capacity, _maze.count());
                    closed.emplace(curr.m_index);

                    for (const auto& neighbour : _maze.get_neighbours(curr.m_index)) {

                        if (const auto& [nActive, nCoord] = neighbour; nActive) {

                            const auto& [n, nDistance] = neighbour;

                            // Check if node is not already visited:
                            if (!closed.contains(n)) {

                                closed.allocate(n, _capacity, _maze.count());
                                closed.emplace(n);

                                // Create a parent node and transfer ownership of 'current' to it. Note: 'current' is now moved!
                                open.emplace(n, curr.m_gScore + static_cast<scalar_t>(1), _h(utils::to_nd(n, _size), _end) * _weight, std::move(curr));
                            }
                        }
                    }
                }
                else { // SOLUTION REACHED ...

                    // Free data which is no longer relevant:
                      open.clear();
                      open.shrink_to_fit();
                    closed.clear();
                    closed.shrink_to_fit();

                    curr.template backtrack<gs_node>(result, _size, curr.m_gScore);

                    break;
                }
            }

            return result;
        }

        [[maybe_unused]]
        std::vector<coord_t> execute(const mazes::grid<Kd, weight_t>& _maze, const coord_t& _start, const coord_t& _end, scalar_t (*_h)(const coord_t&, const coord_t&), const scalar_t& _weight, size_t _capacity) const override {

            std::vector<coord_t> result;

            const auto s = utils::to_1d(_start, _maze.size());
            const auto e = utils::to_1d(_end, _maze.size());

            // Create closed set:
            _capacity = std::max(_capacity, std::max(s, e));
            existence_set closed({ s }, _capacity);

            // Create open set:
            heap<gs_node, typename gs_node::max> open;
            open.emplace(s, static_cast<scalar_t>(0), _h(_start, _end));

            // Main loop:
            while (!open.empty()) {

                auto curr(std::move(open.top()));
                open.pop();

                if (curr.m_index != e) { // SEARCH FOR SOLUTION...

                    closed.allocate(curr.m_index, _capacity, _maze.count());
                    closed.emplace(curr.m_index);

                    for (const auto& neighbour : _maze.get_neighbours(curr.m_index)) {

                        if (const auto& [nActive, nCoord] = neighbour; nActive) {

                            const auto n = utils::to_1d(nCoord, _maze.size());

                            // Check if node is not already visited:
                            if (!closed.contains(n)) {

                                closed.allocate(n, _capacity, _maze.count());
                                closed.emplace(n);

                                // Create a parent node and transfer ownership of 'current' to it. Note: 'current' is now moved!
                                open.emplace(n, curr.m_gScore + static_cast<scalar_t>(1), _h(nCoord, _end) * _weight, std::move(curr));
                            }
                        }
                    }
                }
                else { // SOLUTION REACHED ...

                    // Free data which is no longer relevant:
                      open.clear();
                      open.shrink_to_fit();
                    closed.clear();
                    closed.shrink_to_fit();

                    curr.template backtrack<gs_node>(result, _maze.size(), curr.m_gScore);

                    break;
                }
            }

            return result;
        }

    };

} //chdr::solvers

#endif //CHDR_GSTAR_HPP