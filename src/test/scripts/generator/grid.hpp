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

		template <typename T, size_t Kd, typename... Args>
		static constexpr auto generate(const chdr::coord<size_t, Kd>& _start, chdr::coord<size_t, Kd>& _end, const double& _loops = 0.0, const double& _obstacles = 0.0, const size_t& _seed = -1U, const Args&... _size) {

			static_assert(std::is_integral_v<T>, "Type T must be an integral type.");

            debug::log("(Maze):");

			using backtracking_t = utils::backtracking<Kd>;

			std::array size { _size... };

			const auto maze = backtracking_t::generate(_start, _end, size, _loops, _obstacles, _seed);

			debug::log("\t[FINISHED] \t(~" + chdr::utils::trim_trailing_zeros(std::to_string(chdr::utils::product<size_t>(size) / static_cast<long double>(1000000000.0))) + "b total candidate nodes)");

			if constexpr (std::is_same_v<T, bool>) {
				return chdr::mazes::grid<Kd, T>(size, maze);
			}
			else {

				std::vector<T> nodes;
				nodes.reserve(maze.size());

				for (const auto& node : maze) {
					nodes.emplace_back(
						node == backtracking_t::WALL ?
						std::numeric_limits<T>::max() :
						std::numeric_limits<T>::lowest()
					);
				}

				return chdr::mazes::grid<Kd, T>(size, nodes);
			}
		}
	};

} //test::generator

#endif //TEST_GRID_HPP