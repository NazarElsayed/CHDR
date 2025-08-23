/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef TEST_GRID_HPP
#define TEST_GRID_HPP

#include <chdr.hpp>
#include <debug.hpp>

#include "utils/backtracking.hpp"

namespace test::generator {

    struct grid final {

    private:

        static constexpr size_t null_v = static_cast<size_t>(-1U);

    public:

        template <typename weight_t, typename coord_t, typename scalar_t>
        static constexpr auto generate(const coord_t& _start, coord_t& _end, const coord_t& _size, double _loops = 0.0, double _obstacles = 0.0, size_t _seed = null_v) {

            static_assert(std::is_integral_v<weight_t>, "Type T must be an integral type.");

            using backtracking_t = utils::backtracking<coord_t>;

            debug::log("(Maze):");

            const auto maze = backtracking_t::generate(_start, _end, _size, _loops, _obstacles, _seed);

            debug::log("\t[FINISHED] \t(~" + chdr::utils::trim_trailing_zeros(std::to_string(chdr::utils::product<size_t>(_size) / 1000000000.0L)) + "b total candidate nodes)");

            if constexpr (std::is_same_v<weight_t, bool>) {
                return chdr::mazes::grid<coord_t, weight_t>(_size, maze);
            }
            else {

                std::vector<weight_t> nodes(maze.size());
                std::transform(maze.begin(), maze.end(), nodes.begin(), [](const auto& _node) {
                    return _node == backtracking_t::WALL ?
                        std::numeric_limits<weight_t>::max() :
                        std::numeric_limits<weight_t>::lowest();
                });

                return chdr::mazes::grid<coord_t, weight_t>(_size, std::move(nodes));
            }
        }
    };

} //test::generator

#endif //TEST_GRID_HPP