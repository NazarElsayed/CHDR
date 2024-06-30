#ifndef TEST_MAZE_HPP
#define TEST_MAZE_HPP

#include <chdr.hpp>
#include <iomanip>

#include "utils/Backtracking.hpp"

namespace Test::Generator {

	struct Grid final {

		template <typename T, size_t Kd>
		static constexpr void Display(const CHDR::Coord<size_t, Kd>& _start, const CHDR::Coord<size_t, Kd>& _end, const CHDR::Coord<size_t, Kd>& _size, const CHDR::Mazes::Grid<Kd, T>& _maze) {

			static_assert(              Kd < 3U, "This maze renderer does not support dimensions higher than 2.");
			static_assert(std::is_integral_v<T>, "Maze type must be an integral type."                          );

			std::ostringstream oss;

			constexpr auto* path_str = "  ";
			constexpr auto* wall_str = "██";
			constexpr auto  line_brk = '\n';

			const bool even_width = _size[0U] % 2U == 0U;

			// Add an upper boundary:
			{
				const auto columns = _size[0U] + (even_width ? 1U : 2U);

				for (size_t i = 0U; i < columns; ++i) {
					oss << wall_str;
				}
				oss << line_brk;
			}

			const auto& nodes = _maze.Nodes();
			for (size_t i = 0U; i < nodes.size(); ++i) {

				if (i % _size[0U] == 0U) { oss << wall_str; }

				auto s = CHDR::Utils::To1D(_start, _size);
				auto e = CHDR::Utils::To1D(_end,   _size);

				     if (i == s) { oss << "St"; }
				else if (i == e) { oss << "En"; }
				else {

					auto val = nodes[i].Value();

					if constexpr (std::is_same_v<T, bool>) {
						oss << (val ? wall_str : path_str);
					}
					else {
						oss << std::hex << std::setfill('0') << std::setw(2) << static_cast<unsigned>(val);
					}

					// Handle end of line:
					const bool end_of_line = (Kd == 1 && i == _size[0U] - 1U) || ((i + 1U) % _size[0U] == 0U);

					if (end_of_line) {
					    if (!even_width) {
					        oss << wall_str;
					    }
					    oss << line_brk;
					}

				}
			}

			// Handle the addition of a lower boundary:
			{
				const bool even_height = Kd > 1U && _size[1U] % 2U == 0U;
				if (!even_height) {

					const auto columns = _size[0U] + (even_width ? 1U : 2U);
					for (size_t i = 0U; i < columns; ++i) {
						oss << wall_str;
					}
					oss << line_brk;
				}
			}

			std::cout << oss.str();
		}

		template <typename T, size_t Kd, typename... Args>
		static constexpr auto Generate(const CHDR::Coord<size_t, Kd>& _start, CHDR::Coord<size_t, Kd>& _end, const size_t& _seed = -1U, const Args&... _size) {

			static_assert(std::is_integral_v<T>, "Type T must be an integral type.");

			using Backtracking = Utils::Backtracking<Kd>;

			std::array size { _size... };

			auto maze = Backtracking::Generate(_start, _end, size, _seed);

			std::vector<CHDR::Node<T>> result;
			result.reserve(maze.size());

			for (size_t i = 0U; i < maze.size(); ++i) {
				result.emplace_back(maze[i] == Backtracking::WALL);
			}

			return result;
		}
	};
}

#endif //TEST_MAZE_HPP