#ifndef TEST_DISPLAY_HPP
#define TEST_DISPLAY_HPP

#include <functional>
#include <unordered_set>
#include <vector>

#include <types/Coord.hpp>
#include <mazes/Grid.hpp>

namespace Test {

    template<typename T, const size_t Kd>
    class Display
    {
    private:

        using coord_t = CHDR::Coord<size_t, Kd>;

        static constexpr auto* empty_str = "  ";
        static constexpr auto*  wall_str = "██";

#ifdef _WIN32
        static constexpr auto* start_str = "00";
        static constexpr auto*   end_str = "11";
        static constexpr auto*  path_str = "--";
#else
        static constexpr auto* start_str = "🏠";
        static constexpr auto*   end_str = "🧀";
        static constexpr auto*  path_str = "🐁";
#endif

        static constexpr auto*  line_brk = "\n";

    public:

        static constexpr void DrawMaze(const CHDR::Coord<size_t, Kd>& _start, const CHDR::Coord<size_t,  Kd>& _end,
                                       const CHDR::Coord<size_t, Kd>& _size , const CHDR::Mazes::Grid<Kd, T>& _maze) {

            static_assert(std::is_integral_v<T>, "Maze type must be an integral type.");

#ifdef _WIN32
            SetConsoleOutputCP(CP_UTF8);
#endif

            const auto s = CHDR::Utils::To1D(_start, _size);
            const auto e = CHDR::Utils::To1D(_end,   _size);

            const bool even_width = _size[0U] % 2U == 0U;

            // Add an upper boundary:
            {
                const auto columns = _size[0U] + (even_width ? 1U : 2U);

                for (size_t i = 0U; i < columns; ++i) {
                    std::cout << wall_str;
                }
                std::cout << line_brk;
            }

            const auto& nodes = _maze.Nodes();
            for (size_t i = 0U; i < nodes.size(); ++i) {

                if (i % _size[0U] == 0U) { std::cout << wall_str; }

                if      (i == s) { std::cout << start_str; }
                else if (i == e) { std::cout <<   end_str; }
                else {
                    const auto val = nodes[i].Value();

                    if constexpr (std::is_same_v<T, bool>) {
                        std::cout << (val ? wall_str : empty_str);
                    }
                    else {
                        std::cout << std::hex << std::setfill('0') << std::setw(2) << static_cast<unsigned>(val);
                    }

                    // Handle end of line:
                    if (const bool end_of_line = (Kd == 1U && i == _size[0U] - 1U) || (i + 1U) % _size[0U] == 0U) {
                        if (!even_width) {
                            std::cout << wall_str;
                        }
                        std::cout << line_brk;
                    }
                }
            }

            // Handle the addition of a lower boundary:
            {
                if (const bool even_height = Kd > 1U && _size[1U] % 2U == 0U;
                    !even_height
                ) {

                    const auto columns = _size[0U] + (even_width ? 1U : 2U);
                    for (size_t i = 0U; i < columns; ++i) {
                        std::cout << wall_str;
                    }
                    std::cout << line_brk;
                }
            }
        }

        static constexpr void DrawMaze(const CHDR::Coord<size_t, Kd>& _start, const CHDR::Coord<size_t,  Kd>& _end,
                                       const CHDR::Coord<size_t, Kd>& _size , const CHDR::Mazes::Grid<Kd, T>& _maze,
                                       const std::vector<coord_t>& _path) {

            static_assert(std::is_integral_v<T>, "Maze type must be an integral type.");

            const auto s = CHDR::Utils::To1D(_start, _size);
            const auto e = CHDR::Utils::To1D(_end,   _size);

            std::unordered_set<size_t, std::function<const size_t&(const size_t&)>> path_set(
                _path.size(),
                [](const size_t& _seed) constexpr -> const size_t& { return _seed; }
            );

            for (const auto& item : _path) {
                path_set.emplace(CHDR::Utils::To1D<size_t>(item, _maze.Size()));
            }

            const bool even_width = _size[0U] % 2U == 0U;

            // Add an upper boundary:
            {
                const auto columns = _size[0U] + (even_width ? 1U : 2U);

                for (size_t i = 0U; i < columns; ++i) {
                    std::cout << wall_str;
                }
                std::cout << line_brk;
            }

            const auto& nodes = _maze.Nodes();
            for (size_t i = 0U; i < nodes.size(); ++i) {

                if (i % _size[0U] == 0U) { std::cout << wall_str; }

                if      (i == s) { std::cout << start_str; }
                else if (i == e) { std::cout <<   end_str; }
                else {

                    if (path_set.find(i) != path_set.end()) {
                        std::cout << path_str;
                    }
                    else {
                        const auto val = nodes[i].Value();

                        if constexpr (std::is_same_v<T, bool>) {
                            std::cout << (val ? wall_str : empty_str);
                        }
                        else {
                            std::cout << std::hex << std::setfill('0') << std::setw(2) << static_cast<unsigned>(val);
                        }
                    }

                    // Handle end of line:
                    if (const bool end_of_line = (Kd == 1U && i == _size[0U] - 1U) || (i + 1U) % _size[0U] == 0U) {
                        if (!even_width) {
                            std::cout << wall_str;
                        }
                        std::cout << line_brk;
                    }
                }
            }

            // Handle the addition of a lower boundary:
            {
                if (const bool even_height = Kd > 1U && _size[1U] % 2U == 0U;
                    !even_height
                ) {

                    const auto columns = _size[0U] + (even_width ? 1U : 2U);
                    for (size_t i = 0U; i < columns; ++i) {
                        std::cout << wall_str;
                    }
                    std::cout << line_brk;
                }
            }
        }
    };

}

#endif //TEST_DISPLAY_HPP