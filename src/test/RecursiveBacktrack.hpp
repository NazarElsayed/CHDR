#ifndef TEST_GENMAZE_HPP
#define TEST_GENMAZE_HPP

#include <algorithm>
#include <random>
#include <vector>

template<size_t Kd = 2U>
class RecursiveBacktrack {

public:

    enum Cell : bool {
        PATH = 0U,
        WALL = 1U,
    };

private:

    static constexpr auto GetDirections(const CHDR::Coord<size_t, Kd>& _coord, const CHDR::Coord<size_t, Kd>& _size, const size_t& _multiplier = 1U) {

        using int_t = long;

        std::vector<CHDR::Coord<int_t, Kd>> result;

        // Positive directions:
        for (size_t i = 0U; i < Kd; ++i) {

            CHDR::Coord<int_t, Kd> dir{};

            if (_coord[i] < _size[i] - _multiplier) {
                dir[i] += _multiplier;

                result.emplace_back(dir);
            }
        }

        // Negative directions:
        for (size_t i = 0U; i < Kd; ++i) {

            CHDR::Coord<int_t, Kd> dir{};

            if (_coord[i] >= _multiplier) {
                dir[i] -= _multiplier;

                result.emplace_back(dir);
            }
        }

        return result;
    }

    static constexpr auto GetNeighbors(const CHDR::Coord<size_t, Kd>& _coord, const CHDR::Coord<size_t, Kd>& _size, const size_t& _multiplier = 1U) {

        std::vector<CHDR::Coord<size_t, Kd>> result;

        // Positive directions:
        for (size_t i = 0U; i < Kd; ++i) {

            auto n = _coord;

            auto& element = n[i];

            if (element < _size[i] - 1U * _multiplier) {
                element += 1U * _multiplier;

                result.emplace_back(n);
            }
        }

        // Negative directions:
        for (size_t i = 0U; i < Kd; ++i) {

            auto n = _coord;

            auto& element = n[i];

            if (element >= _multiplier) {
                element -= _multiplier;

                result.emplace_back(n);
            }
        }

        return result;
    }

    static void GOCarveFrom(CHDR::Coord<size_t, Kd> _coord, const CHDR::Coord<size_t, Kd>& _size, std::vector<Cell> &_grid) {
        CarveFrom(_coord, _size, _grid);
    }

    static void CarveFrom(const CHDR::Coord<size_t, Kd>& _coord, const CHDR::Coord<size_t, Kd>& _size, std::vector<Cell> &_grid) {

        _grid[CHDR::Utils::To1D(_coord, _size)] = PATH;

        auto dirs = GetDirections(_coord, _size, 1U);

        std::random_device rd;
        std::mt19937 g(rd());
        std::shuffle(dirs.begin(), dirs.end(), g);

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

                    CarveFrom(cc, _size, _grid);
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
    static auto Generate(const size_t& _seed, CHDR::Coord<size_t, Kd> _size) {

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

                CHDR::Coord<size_t, Kd> start{0U, 0U};

                GOCarveFrom(start, _size, result);
            }
            catch (const std::exception& e) {
                Debug::Log(e);
            }
        }

        return result;
    }

};

#endif //TEST_GENMAZE_HPP