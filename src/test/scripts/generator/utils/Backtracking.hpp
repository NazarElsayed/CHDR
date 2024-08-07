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
            stack.push({ _coord, 0U });

            while (!stack.empty()) {

                auto [currentCoord, depth] = stack.top();
                _grid[CHDR::Utils::To1D(currentCoord, _size)] = PATH;

                if (depth > _farthest.second) {
                    _farthest.first = currentCoord;
                    _farthest.second = depth;
                }

                auto dirs = GetDirections(currentCoord, _size);
                std::shuffle(dirs.begin(), dirs.end(),_random);

                bool hasUnvisited = false;
                for (const auto& [inBounds, dir] : dirs) {

                    if (inBounds) {

                        auto lc = currentCoord;
                        auto cc = currentCoord;

                        bool validCellNeighbor = true;

                        // as in original function
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

                                stack.push({ cc, depth + 1 });

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
         * Generate a maze using the backtracking algorithm.
         *
         * @param _seed The seed used for random number generation. Set to -1 to use a random seed.
         * @param _size The size of the maze.
         * @return A vector representing the generated maze.
         *
         * @see Buck, J., 2010. Buckblog: Maze Generation: Recursive Backtracking. The Buckblog [online]. Available from: https://weblog.jamisbuck.org/2010/12/27/maze-generation-recursive-backtracking [Accessed 1 Jul 2024].
         */
        static constexpr auto Generate(const coord_t& _start, coord_t& _end, const coord_t& _size, const size_t& _seed) {

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
            result.resize(product, WALL);

            // Maze algo:
            try {

                std::mt19937 gen(_seed == null_v ? std::random_device().operator()() : _seed);

                std::pair<coord_t, size_t> farthest { _start, 0U };

                CarveFrom(_start, farthest, _size, result, gen);

                _end = farthest.first;
            }
            catch (const std::exception& e) {
                Debug::Log(e);

                _end = _start;
            }

            return result;
        }

    };

    #endif //TEST_GENMAZE_HPP
}