#ifndef TEST_GENMAZE_HPP
#define TEST_GENMAZE_HPP

#include <algorithm>
#include <random>
#include <vector>

namespace Test::Generator::Utils {

    template<size_t Kd = 2U>
    class Backtracking {

    public:

        enum Cell : bool {
            PATH = 0U,
            WALL = 1U,
        };

    private:

        static constexpr auto GetDirections(const CHDR::Coord<size_t, Kd>& _coord, const CHDR::Coord<size_t, Kd>& _size) {

            constexpr size_t step(1);

            std::vector<CHDR::Coord<size_t, Kd>> result;

            // Positive:
            for (size_t i = 0U; i < Kd; ++i) {

                CHDR::Coord<size_t, Kd> dir{};

                if (_coord[i] < _size[i] - step) {
                    dir[i] += step;

                    result.emplace_back(dir);
                }
            }

            // Negative:
            for (size_t i = 0U; i < Kd; ++i) {

                CHDR::Coord<size_t, Kd> dir{};

                if (_coord[i] >= step) {
                    dir[i] -= step;

                    result.emplace_back(dir);
                }
            }

            return result;
        }

        static constexpr void CarveFrom(const CHDR::Coord<size_t, Kd>& _coord, std::pair<CHDR::Coord<size_t, Kd>, size_t>& _farthest, const size_t& _depth, const CHDR::Coord<size_t, Kd>& _size, std::vector<Cell>& _grid, std::mt19937 _random) {

            _grid[CHDR::Utils::To1D(_coord, _size)] = PATH;

            if (_depth > _farthest.second) {
                _farthest.first  = _coord;
                _farthest.second = _depth;
            }

            auto dirs = GetDirections(_coord, _size);

            std::shuffle(dirs.begin(), dirs.end(), _random);

            // Attempt to find unvisited cells:
            for (size_t i = 0U; i < dirs.size(); ++i) {

                const auto& dir = dirs[i];

                auto lc = _coord;
                auto cc = _coord;

                bool validCellNeighbor = true;

                for (size_t j = 0U; j < Kd; ++j) {

                    lc[j] += dir[j];
                    cc[j] += dir[j] * 2U;

                    if (cc[j] >= _size[j]) {
                        validCellNeighbor = false;

                        break;
                    }
                }

                if (validCellNeighbor) {

                    auto& ln = _grid[CHDR::Utils::To1D(lc, _size)];
                    auto& cn = _grid[CHDR::Utils::To1D(cc, _size)];

                    if (cn == WALL) {
                        ln = PATH;

                        CarveFrom(cc, _farthest, _depth + 1, _size, _grid, _random);
                    }
                }
            }
        }

    public:

        /**
         * Generate a maze using the Ellers algorithm.
         *
         * @param _seed The seed used for random number generation. Set to -1 to use a random seed.
         * @param _size The size of the maze.
         * @return A vector representing the generated maze.
         */
        static auto Generate(const CHDR::Coord<size_t, Kd>& _start, CHDR::Coord<size_t, Kd>& _end, CHDR::Coord<size_t, Kd> _size, const size_t& _seed) {

            static constexpr size_t null_v = -1U;

            std::vector<Cell> result;

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
             */

            // Attempt to allocate the desired amount of space in memory.
            const auto product = CHDR::Utils::Product<size_t>(_size);
            if (product > 0U) {

                // TODO: Ensure that product does not overflow!

                result.resize(product, WALL);

                // Maze algo:
                try {

                    std::mt19937 gen(_seed == null_v ? std::random_device().operator()() : _seed);

                    std::pair<CHDR::Coord<size_t, Kd>, size_t> farthest { _start, 0 };

                    CarveFrom(_start, farthest, 0, _size, result, gen);

                    _end = farthest.first;
                }
                catch (const std::exception& e) {
                    Debug::Log(e);

                    _end = _start;
                }
            }

            return result;
        }

    };

    #endif //TEST_GENMAZE_HPP
}