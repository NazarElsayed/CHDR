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

#include "Debug.hpp"

#include "utils/Backtracking.hpp"

namespace Test::Generator {

	struct Grid final {

		template <typename T, const size_t Kd, typename... Args>
		static constexpr auto Generate(const CHDR::Coord<size_t, Kd>& _start, CHDR::Coord<size_t, Kd>& _end, const double& _loops = 0.0, const double& _obstacles = 0.0, const size_t& _seed = -1U, const Args&... _size) {

			static_assert(std::is_integral_v<T>, "Type T must be an integral type.");

            Debug::Log("(Maze):");

			using Backtracking = Utils::Backtracking<Kd>;

			std::array size { _size... };

			auto maze = Backtracking::Generate(_start, _end, size, _loops, _obstacles, _seed);

			std::vector<CHDR::Mazes::WeightedNode<T>> nodes;
            nodes.reserve(maze.size());

            while (!maze.empty()) {
                nodes.emplace_back(maze.back() == Backtracking::WALL);
                maze.pop_back();
            }

            std::reverse(nodes.begin(), nodes.end());

            const auto result = CHDR::Mazes::Grid<Kd, T>(size, nodes);

            Debug::Log("\t[FINISHED] \t(~" + CHDR::Utils::Trim_Trailing_Zeros(std::to_string(CHDR::Utils::Product<size_t>(size) / static_cast<long double>(1000000000.0))) + "b total candidate nodes)");

			return result;
		}
	};
}

#endif //TEST_GRID_HPP