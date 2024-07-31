#ifndef TEST_DISPLAY_HPP
#define TEST_DISPLAY_HPP

#include <vector>

#include <types/Coord.hpp>
#include <mazes/Grid.hpp>

namespace Test {

    template<typename T, size_t Kd>
    class Display {

    private:

        using coord_t = CHDR::Coord<size_t, Kd>;

    public:

        static constexpr void DrawMaze(const CHDR::Coord<size_t, Kd>& _start, const CHDR::Coord<size_t,  Kd>& _end,
                                       const CHDR::Coord<size_t, Kd>& _size , const CHDR::Mazes::Grid<Kd, T>& _maze) {

            static_assert(Kd < 3U, "This maze renderer does not support dimensions higher than 2.");
            static_assert(std::is_integral_v<T>, "Maze type must be an integral type.");

            std::ostringstream oss;

            constexpr auto* empty_str = "  ";
            constexpr auto*  wall_str = "██";
            constexpr auto*  line_brk = "\n";

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
                auto e = CHDR::Utils::To1D(_end, _size);

                if (i == s) { oss << "St"; }
                else if (i == e) { oss << "En"; }
                else {
                    auto val = nodes[i].Value();

                    if constexpr (std::is_same_v<T, bool>) {
                        oss << (val ? wall_str : empty_str);
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
        }

        static constexpr void DrawMaze(const CHDR::Coord<size_t, Kd>& _start, const CHDR::Coord<size_t,  Kd>& _end,
                                       const CHDR::Coord<size_t, Kd>& _size , const CHDR::Mazes::Grid<Kd, T>& _maze,
                                       const std::vector<coord_t>&    _path) {

            static_assert(Kd < 3U, "This maze renderer does not support dimensions higher than 2.");
            static_assert(std::is_integral_v<T>, "Maze type must be an integral type.");

            std::ostringstream oss;
            std::vector<std::string> image;

            image.reserve(CHDR::Utils::To1D(_maze.Size(), _maze.Size()));

            constexpr const auto* const empty_str = "  ";
            constexpr const auto* const  wall_str = "██";
            constexpr const auto* const  line_brk = "\n";

            const bool even_width = _size[0U] % 2U == 0U;

            std::cout << std::endl;

            // Add an upper boundary:
            {
                const auto columns = _size[0U] + (even_width ? 1U : 2U);

                for (size_t i = 0U; i < columns; ++i) {
                    image.emplace_back(wall_str);
                }
                image.emplace_back(line_brk);
            }

            const auto& nodes = _maze.Nodes();
            for (size_t i = 0U; i < nodes.size(); ++i) {
                if (i % _size[0U] == 0U) {
                    image.emplace_back(wall_str);
                }

                auto s = CHDR::Utils::To1D(_start, _size);
                auto e = CHDR::Utils::To1D(_end, _size);

                if (i == s) {
                    image.emplace_back("🏠");
                }
                else if (i == e) {
                    image.emplace_back("🧀");
                }
                else {
                    auto val = nodes[i].Value();

                    if constexpr (std::is_same_v<T, bool>) {
                        image.emplace_back(val ? wall_str : empty_str);
                    }
                    else {

                        std::ostringstream hex;
                        hex << std::hex << std::setfill('0') << std::setw(2) << static_cast<unsigned>(val);

                        image.emplace_back(hex.str());
                    }

                    // Handle end of line:
                    const bool end_of_line = (Kd == 1 && i == _size[0U] - 1U) || ((i + 1U) % _size[0U] == 0U);

                    if (end_of_line) {
                        if (!even_width) {
                            image.emplace_back(wall_str);
                        }
                        image.emplace_back(line_brk);
                    }
                }
            }

            // Handle the addition of a lower boundary:
            {
                const bool even_height = Kd > 1U && _size[1U] % 2U == 0U;
                if (!even_height) {

                    const auto columns = _size[0U] + (even_width ? 1U : 2U);
                    for (size_t i = 0U; i < columns; ++i) {
                        image.emplace_back(wall_str);
                    }
                    image.emplace_back(line_brk);
                }
            }

            auto size = _maze.Size();
            size[0] += even_width ? 2 : 3;

            const int offset = size[0] + 1;
            for (auto coord : _path) {

                if (coord != _end) {
                    auto index   = CHDR::Utils::To1D(coord, size) + offset;
                    image[index] = "🐁";
                }
            }

            for (const auto& item : image) {
                std::cout << item;
            }
        }
    };
}

#endif //TEST_DISPLAY_HPP
