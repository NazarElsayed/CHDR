/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef TEST_DISPLAY_HPP
#define TEST_DISPLAY_HPP

#include <vector>

#include <mazes/Grid.hpp>
#include <types/Coord.hpp>
#include <types/existence_set.hpp>

namespace test {

    template<typename T, const size_t Kd>
    class display {

    private:

        using coord_t = chdr::coord_t<size_t, Kd>;

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

        static constexpr void drawMaze(const chdr::coord_t<size_t, Kd>& _start, const chdr::coord_t<size_t,  Kd>& _end,
                                       const chdr::coord_t<size_t, Kd>& _size , const chdr::mazes::Grid<Kd, T>& _maze) {

            static_assert(std::is_integral_v<T>, "Maze type must be an integral type.");

#ifdef _WIN32
            SetConsoleOutputCP(CP_UTF8);
#endif

            const auto s = chdr::Utils::To1D(_start, _size);
            const auto e = chdr::Utils::To1D(_end, _size);

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

        static constexpr void drawMaze(const chdr::coord_t<size_t, Kd>& _start, const chdr::coord_t<size_t,  Kd>& _end,
                                       const chdr::coord_t<size_t, Kd>& _size , const chdr::mazes::Grid<Kd, T>& _maze,
                                       const std::vector<coord_t>& _path) {

            static_assert(std::is_integral_v<T>, "Maze type must be an integral type.");

            const auto s = chdr::Utils::To1D(_start, _size);
            const auto e = chdr::Utils::To1D(_end, _size);

            chdr::existence_set path_set(_path.size());

            for (const auto& item : _path) {
                path_set.add(chdr::Utils::To1D<size_t>(item, _maze.Size()));
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

                    if (path_set.contains(i)) {
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