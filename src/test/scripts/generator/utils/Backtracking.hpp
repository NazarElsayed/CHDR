#ifndef TEST_GENMAZE_HPP
#define TEST_GENMAZE_HPP

#include <algorithm>
#include <random>
#include <stack>
#include <vector>

namespace Test::Generator::Utils {

    template<const size_t Kd = 2U>
    class Backtracking {

        using coord_t = CHDR::Coord<size_t, Kd>;

    public:

        enum Cell : bool {
            PATH = 0U,
            WALL = 1U,
        };

    private:

        static constexpr auto GetDirections(const coord_t& _coord, const coord_t& _size) {

            constexpr size_t step(1U);

            std::array<std::pair<bool, CHDR::Coord<size_t, Kd>>, Kd * 2U> result;

            // Positive:
            for (size_t i = 0U; i < Kd; ++i) {

                coord_t dir{};
                dir[i] += step;

                result[i] = { _coord[i] < _size[i] - step, dir };
            }

            // Negative:
            for (size_t i = 0U; i < Kd; ++i) {

                coord_t dir{};
                dir[i] -= step;

                result[Kd + i] = { _coord[i] >= step, dir };
            }

            return result;
        }

        static constexpr void CarveFrom(const coord_t& _coord, std::pair<coord_t, size_t>& _farthest, const coord_t& _size, std::vector<Cell>& _grid, std::mt19937 _random) {

            std::stack<std::pair<coord_t, size_t>> stack;
            stack.emplace(_coord, 0U);

            while (!stack.empty()) {

                auto [currentCoord, depth] = stack.top();
                _grid[CHDR::Utils::To1D(currentCoord, _size)] = PATH;

                if (depth > _farthest.second) {
                    _farthest.first = currentCoord;
                    _farthest.second = depth;
                }

                auto dirs = GetDirections(currentCoord, _size);
                std::shuffle(dirs.begin(), dirs.end(), _random);

                bool hasUnvisited = false;
                for (const auto& [inBounds, dir] : dirs) {

                    if (inBounds) {

                        auto lc = currentCoord;
                        auto cc = currentCoord;

                        bool validCellNeighbor = true;

                        for (size_t j = 0U; j < Kd; ++j) {

                            lc[j] +=      dir[j];
                            cc[j] += 2U * dir[j];

                            if (cc[j] >= _size[j]) {
                                validCellNeighbor = false;

                                break;
                            }
                        }

                        if (validCellNeighbor) {

                                  auto& ln = _grid[CHDR::Utils::To1D(lc, _size)];
                            const auto& cn = _grid[CHDR::Utils::To1D(cc, _size)];

                            if (cn == WALL) {
                                ln = PATH;

                                stack.emplace(cc, depth + 1);

                                hasUnvisited = true;

                                break;
                            }
                        }
                    }
                }

                if (!hasUnvisited) {
                    stack.pop();
                }
            }
        }

    public:

        /**
         * Generates a maze using the backtracking algorithm.
         *
         * @param [in] _start The starting coordinates in the field.
         * @param [in] _end The end coordinates in the field (updated by the method).
         * @param [in] _size The size of the 2D maze grid.
         * @param [in] _seed The seed value for the random number generator.
         *
         * @param [in] _loops (optional) Value between 0.0 and 1.0 indicating the possibility of the maze containing loops.
         *                    The default value is 0.0, which yields no loops.
         *
         * @param [in] _obstacles (optional) Value between 0.0 and 1.0 indicating the possibility of the maze containing obstacles.
         *                        The presence of obstacles may make some paths unsolvable.
         *                        The default value is 0.0, which yields no obstacles.
         *
         * @return A vector representing the maze grid.
         *
         * @see Buck, J., 2010. Buckblog: Maze Generation: Recursive Backtracking. The Buckblog [online]. Available from: https://weblog.jamisbuck.org/2010/12/27/maze-generation-recursive-backtracking [Accessed 1 Jul 2024].
         */
        static constexpr auto Generate(const coord_t& _start, coord_t& _end, const coord_t& _size, const float& _loops = 0.0F, const float& _obstacles = 0.0F, const size_t& _seed = -1U) {

            /*
             * 1. Choose a starting point in the field.
             *
             * 2. Randomly choose a wall at that point and carve a passage through to the adjacent cell,
             *    but only if the adjacent cell has not been visited yet.
             *    This becomes the new current cell.
             *
             * 3. If all adjacent cells have been visited,
             *    back up to the last cell that has uncarved walls and repeat.
             *
             * 4. The algorithm ends when the process has backed all the way up to the starting point.
             *
             * (Buck, 2010)
             */

            constexpr size_t null_v = -1U;

            // Attempt to allocate the desired amount of space in memory.
            const auto product = CHDR::Utils::Product<size_t>(_size);

            // TODO: Ensure that product does not overflow!

            std::vector<Cell> result;

#if __cplusplus >= 202002L
            try {
#endif // __cplusplus >= 202002L

                // Maze algo:
                const auto seed = _seed == null_v ? std::random_device().operator()() : _seed;
                std::mt19937 gen(seed);

                Debug::Log("Maze: Seed (" + std::to_string(seed) + ")");

                result.resize(product, WALL);

                std::pair<coord_t, size_t> farthest { _start, 0U };

                // Carve a maze using the recursive backtracking algorithm:
                CarveFrom(_start, farthest, _size, result, gen);

                if (_loops > 0.0F || _obstacles > 0.0F) {

                    // Randomly knock down walls if the maze is meant to contain loops:
                    for (size_t i = 1U; i < result.size(); ++i) {

                        auto c = CHDR::Utils::ToND<size_t, Kd>(i, _size);

                        bool link = false;
                        for (size_t j = 0U; j < Kd; ++j) {

                             if (c[j] % 2U == 0U) {
                                 link = true;

                                 break;
                             }
                        }

                        if (link) {

                            bool edge = false;

                            for (size_t k = 0U; k < Kd; ++k) {
                                if (c[k] >= _size[k] - 1U) {
                                    edge = true;

                                    break;
                                }
                            }

                            if (!edge) {

                                const auto obstacle_chance = static_cast<float>(gen()) / static_cast<float>(std::mt19937::max());

                                if (obstacle_chance < _obstacles) {
                                    result[CHDR::Utils::To1D<size_t>(c, _size)] = WALL;
                                }
                                else {
                                    const auto loop_chance = static_cast<float>(gen()) / static_cast<float>(std::mt19937::max());

                                    if (loop_chance < _loops) {
                                        result[CHDR::Utils::To1D<size_t>(c, _size)] = PATH;
                                    }
                                }
                            }
                        }
                    }
                }

                _end = farthest.first;

#if __cplusplus >= 202002L
            }
            catch (const std::exception& e) {
                Debug::Log(e);

                _end = _start;
            }
#endif // __cplusplus >= 202002L

            return result;
        }

    };

    #endif //TEST_GENMAZE_HPP
}