#ifndef TEST_MAZE_HPP
#define TEST_MAZE_HPP

#include <chdr.hpp>

#include "utils/Backtracking.hpp"

namespace Test::Generator {

	struct Grid final {

		template <typename T, const size_t Kd, typename... Args>
		static constexpr auto Generate(const CHDR::Coord<size_t, Kd>& _start, CHDR::Coord<size_t, Kd>& _end, const float& _loops = 0.0F, const float& _obstacles = 0.0f, const size_t& _seed = -1U, const Args&... _size) {

			static_assert(std::is_integral_v<T>, "Type T must be an integral type.");

            Debug::Log("(Maze):");

			using Backtracking = Utils::Backtracking<Kd>;

			std::array size { _size... };

			auto maze = Backtracking::Generate(_start, _end, size, _loops, _obstacles, _seed);

			std::vector<CHDR::Node<T>> result;
			result.reserve(maze.size());

			for (size_t i = 0U; i < maze.size(); ++i) {
				result.emplace_back(maze[i] == Backtracking::WALL);
			}

            Debug::Log("\t[FINISHED] \t(~" + CHDR::Utils::Trim_Trailing_Zeros(std::to_string(CHDR::Utils::Product<size_t>(size) / static_cast<long double>(1000000000.0))) + "b total candidate nodes)");

			return result;
		}
	};
}

#endif //TEST_MAZE_HPP