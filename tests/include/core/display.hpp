/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef TEST_DISPLAY_HPP
#define TEST_DISPLAY_HPP

#include <chdr.hpp>

#include <cstddef>
#include <vector>

namespace test {

    class display {

    private:

        static constexpr auto* s_empty_str = "  ";
        static constexpr auto*  s_wall_str = "██";

#ifdef _WIN32
        static constexpr auto* start_str = "00";
        static constexpr auto*   end_str = "11";
        static constexpr auto*  path_str = "--";
#else
        static constexpr auto* s_start_str = "🏠";
        static constexpr auto*   s_end_str = "🧀";
        static constexpr auto*  s_path_str = "🐁";
#endif

        static constexpr auto*  s_line_brk = "\n";

    public:

        template<typename weight_t, typename coord_t>
        static constexpr void draw_maze(const coord_t& _start, const coord_t& _end, const coord_t& _size,  const chdr::mazes::grid<coord_t, weight_t>& _maze) {

            static_assert(std::is_integral_v<weight_t>, "Maze type must be an integral type.");

#ifdef _WIN32
            SetConsoleOutputCP(CP_UTF8);
#endif

            const auto s = chdr::utils::to_1d(_start, _size);
            const auto e = chdr::utils::to_1d(_end, _size);

            const bool even_width = _size[0U] % 2U == 0U;

            // Add an upper boundary:
            {
                const auto columns = _size[0U] + (even_width ? 1U : 2U);

                for (size_t i = 0U; i < columns; ++i) {
                    std::cout << s_wall_str;
                }
                std::cout << s_line_brk;
            }

            const auto& nodes = _maze.nodes();
            for (size_t i = 0U; i < nodes.size(); ++i) {

                if (i % _size[0U] == 0U) { std::cout << s_wall_str; }

                if      (i == s) { std::cout << s_start_str; }
                else if (i == e) { std::cout << s_end_str; }
                else {
                    const auto val = nodes[i].Value();

                    if constexpr (std::is_same_v<weight_t, bool>) {
                        std::cout << (val ? s_wall_str : s_empty_str);
                    }
                    else {
                        std::cout << std::hex << std::setfill('0') << std::setw(2) << static_cast<unsigned>(val);
                    }

                    // Handle end of line:
                    if ([[maybe_unused]] const bool end_of_line = (_maze.s_rank == 1U && i == _size[0U] - 1U) || (i + 1U) % _size[0U] == 0U) {
                        if (!even_width) {
                            std::cout << s_wall_str;
                        }
                        std::cout << s_line_brk;
                    }
                }
            }

            // Handle the addition of a lower boundary:
            {
                if (const bool even_height = _maze.s_rank > 1U && _size[1U] % 2U == 0U;
                    !even_height
                ) {

                    const auto columns = _size[0U] + (even_width ? 1U : 2U);
                    for (size_t i = 0U; i < columns; ++i) {
                        std::cout << s_wall_str;
                    }
                    std::cout << s_line_brk;
                }
            }
        }

        template<typename weight_t, typename coord_t>
        static constexpr void draw_maze(const coord_t& _start, const coord_t& _end, const coord_t& _size,  const chdr::mazes::grid<coord_t, weight_t>& _maze, const std::vector<coord_t>& _path) {

            static_assert(std::is_integral_v<weight_t>, "Maze type must be an integral type.");

            const auto s = chdr::utils::to_1d(_start, _size);
            const auto e = chdr::utils::to_1d(_end, _size);

            chdr::existence_set path_set(_path.size());

            for (const auto& item : _path) {
                path_set.push(chdr::utils::to_1d(item, _maze.size()));
            }

            const bool even_width = _size[0U] % 2U == 0U;

            // Add an upper boundary:
            {
                const auto columns = _size[0U] + (even_width ? 1U : 2U);

                for (size_t i = 0U; i < columns; ++i) {
                    std::cout << s_wall_str;
                }
                std::cout << s_line_brk;
            }

            size_t i = 0U;
            for (auto node : _maze) {

                if (i % _size[0U] == 0U) { std::cout << s_wall_str; }

                if      (i == s) { std::cout << s_start_str; }
                else if (i == e) { std::cout << s_end_str; }
                else {

                    if (path_set.contains(i)) {
                        std::cout << s_path_str;
                    }
                    else {
                        const auto val = node;

                        if constexpr (std::is_same_v<weight_t, bool>) {
                            std::cout << (val ? s_wall_str : s_empty_str);
                        }
                        else {
                            if (val == std::numeric_limits<weight_t>::lowest()) {
                                std::cout << s_empty_str;
                            }
                            else if (val == std::numeric_limits<weight_t>::max()) {
                                std::cout << s_wall_str;
                            }
                            else {
                                std::cout << std::hex << std::setfill('0') << std::setw(2) << static_cast<unsigned>(val);
                            }
                        }
                    }

                    // Handle end of line:
                    if ([[maybe_unused]] const bool end_of_line = (_maze.s_rank == 1U && i == _size[0U] - 1U) || (i + 1U) % _size[0U] == 0U) {
                        if (!even_width) {
                            std::cout << s_wall_str;
                        }
                        std::cout << s_line_brk;
                    }
                }
                ++i;
            }

            // Handle the addition of a lower boundary:
            {
                if (const bool even_height = _maze.s_rank > 1U && _size[1U] % 2U == 0U;
                    !even_height
                ) {

                    const auto columns = _size[0U] + (even_width ? 1U : 2U);
                    for (size_t j = 0U; i < columns; ++j) {
                        std::cout << s_wall_str;
                    }
                    std::cout << s_line_brk;
                }
            }
        }
    };

} //test

#endif //TEST_DISPLAY_HPP