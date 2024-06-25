#ifndef TEST_GENMAZE_HPP
#define TEST_GENMAZE_HPP

#include <vector>
#include <unordered_map>
#include <algorithm>
#include <random>

class Ellers {

public:

    enum Cell : bool {
        PATH = false,
        WALL = true,
    };

    /**
     * Generate a maze using the Ellers algorithm.
     *
     * @param _seed The seed used for random number generation. Set to -1 to use a random seed.
     * @param _width The width of the maze.
     * @param _height The height of the maze.
     * @return A vector representing the generated maze.
     */
    static auto Generate(size_t _seed, size_t _width, size_t _height) {

        std::vector maze(_height, std::vector(_width, Cell::WALL));

        std::unordered_map<size_t, std::vector<size_t>> sets;
        std::vector<size_t> currentRowSets(_width, -1);

        std::mt19937 gen(_seed == -1 ? std::random_device().operator()() : _seed);

        for (size_t row = 0; row < _height; row += 2) {

            // Assign sets to cells in a row.
            for (size_t col = 0; col < _width; col += 2) {

                if (row == 0 || currentRowSets[col] == -1) {
                    currentRowSets[col] = col;
                    sets[currentRowSets[col]] = { col };
                }

                maze[row][col] = Cell::PATH;

                // Randomly join adjacent.
                if (col < _width - 2 && (row == _height - 2 || gen() % 2)) {
                    currentRowSets[col + 2] = currentRowSets[col];
                    sets[currentRowSets[col]].emplace_back(col + 2);
                    maze[row][col + 1] = Cell::PATH;
                }

                // Randomly create vertical connections.
                if (row < _height - 2 && (col >= _width - 2 || gen() % 2)) {
                    maze[row + 1][col] = Cell::PATH;
                    currentRowSets[col] = -1;
                }
            }

            // Ensure that every set has at least one vertical connection.
            for (const auto& item : sets) {

                if (find(currentRowSets.begin(), currentRowSets.end(), item.first) == currentRowSets.end()) {

                    const auto& randomCol = item.second[gen() % item.second.size()];

                    maze[row + 1][randomCol] = Cell::PATH;
                    currentRowSets[randomCol] = -1;
                }
            }
        }

        // Flatten result.
        std::vector<Cell> result;
        result.reserve(_width * _height);

        for (auto& item : maze) {
            CHDR::Utils::MoveInto(std::move(item), result);
        }

        return result;
    }
};

#endif //TEST_GENMAZE_HPP