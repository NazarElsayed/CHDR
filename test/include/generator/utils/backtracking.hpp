/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef TEST_BACKTRACKING_HPP
#define TEST_BACKTRACKING_HPP

#include <algorithm>
#include <cstddef>
#include <exception>
#include <vector>

#include "lcg.hpp"

namespace test::generator::utils {

    template<typename coord_t>
    class backtracking {

        static constexpr auto Kd = std::tuple_size_v<std::decay_t<coord_t>>;

        using rng_engine_t = lcg<size_t>;

    public:
        static constexpr bool PATH { false };
        static constexpr bool WALL { true  };

    private:

        static constexpr size_t null_v = static_cast<size_t>(-1U);

        static constexpr bool valid_dimensionality(const coord_t& _size) {
            return std::all_of(_size.begin(), _size.end(), [](const auto _item) { return _item != 0U; });
        }

        static constexpr bool is_link(const coord_t& _coord) {
            return std::any_of(_coord.begin(), _coord.end(), [](const auto _item) { return _item % 2U == 0U; });
        }

        static constexpr bool is_edge(const coord_t& _coord, const coord_t& _size) {

            bool result = false;

            for (size_t k = 0U; k < Kd; ++k) {
                if (_coord[k] >= _size[k] - 1U) {
                    result = true;
                    break;
                }
            }

            return result;
        }

        template <size_t Index>
        static constexpr void compute_single_axis(const coord_t& _coord, const coord_t& _size, std::pair<bool, coord_t>& negOutput, std::pair<bool, coord_t>& posOutput) noexcept {

            constexpr size_t step(1U);

            coord_t dir{};

            dir[Index] = step;
            negOutput = { _coord[Index] < _size[Index] - step, dir };

            dir[Index] = -step;
            posOutput = { _coord[Index] >= step, dir };
        }

        template <size_t... Indices>
        static constexpr auto compute_axis_neighbours(const coord_t& _coord, const coord_t& _size, std::index_sequence<Indices...>) {

            std::array<std::pair<bool, coord_t>, Kd * 2U> result;
            (compute_single_axis<Indices>(_coord, _size, result[Indices], result[Kd + Indices]), ...);

            return result;
        }

        static constexpr auto get_directions(const coord_t& _coord, const coord_t& _size) {
            return compute_axis_neighbours(_coord, _size, std::make_index_sequence<Kd>{});
        }

        template <typename it>
        static constexpr void constexpr_shuffle(it _first, it _last, rng_engine_t& _rng) noexcept {

            using diff_t = typename std::iterator_traits<it>::difference_type;

            const diff_t length = (_last - _first) - 1;

            for (diff_t i = length; i > 0; --i) {
                std::iter_swap(_first + i, _first + static_cast<diff_t>(_rng() % length));
            }
        }

        template <typename container_t>
        static
#if defined(__cpp_constexpr_dynamic_alloc) && (__cpp_constexpr_dynamic_alloc >= 201907L)
        constexpr
#endif // defined(__cpp_constexpr_dynamic_alloc) && (__cpp_constexpr_dynamic_alloc >= 201907L)
        void carve_from(const coord_t& _coord, std::pair<coord_t, size_t>& _farthest, const coord_t& _size, container_t& _grid, rng_engine_t& _rng) {

            static_assert(std::is_invocable_v<decltype(static_cast<typename container_t::reference (container_t::*)(size_t)>(&container_t::operator[])),
                          container_t&, size_t>,
                          "container_t must implement the subscript operator []");

            chdr::stack<std::pair<coord_t, size_t>> stack;
            stack.emplace(_coord, 0U);

            while (!stack.empty()) {

                auto& [currentCoord, depth] = stack.top();
                _grid[chdr::utils::to_1d(currentCoord, _size)] = PATH;

                if (depth > _farthest.second) {
                    _farthest.first = currentCoord;
                    _farthest.second = depth;
                }

                auto dirs = get_directions(currentCoord, _size);
                std::shuffle(dirs.begin(), dirs.end(), _rng);
                //constexpr_shuffle(dirs.begin(), dirs.end(), _rng);

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

                        if (validCellNeighbor && _grid[chdr::utils::to_1d(cc, _size)] == WALL) {

                            _grid[chdr::utils::to_1d(lc, _size)] = PATH;
                            stack.emplace(cc, depth + 1U);
                            hasUnvisited = true;
                            break;
                        }
                    }
                }
                if (!hasUnvisited) { stack.pop(); }
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
         * @param [in] _loops (optional) ID between 0.0 and 1.0 indicating the possibility of the maze containing loops.
         *                    The default value is 0.0, which yields no loops.
         *
         * @param [in] _obstacles (optional) ID between 0.0 and 1.0 indicating the possibility of the maze containing obstacles.
         *                        The presence of obstacles may make some paths unsolvable.
         *                        The default value is 0.0, which yields no obstacles.
         *
         * @return A vector representing the maze grid.
         *
         * @see Buck, J., 2010. Buckblog: Maze Generation: Recursive Backtracking. The Buckblog [online]. Available from: https://weblog.jamisbuck.org/2010/12/27/maze-generation-recursive-backtracking [Accessed 1 Jul 2024].
         */
        static
#if defined(__cpp_constexpr_dynamic_alloc) && (__cpp_constexpr_dynamic_alloc >= 201907L)
        constexpr
#endif // defined(__cpp_constexpr_dynamic_alloc) && (__cpp_constexpr_dynamic_alloc >= 201907L)
        auto generate(const coord_t& _start, coord_t& _end, const coord_t& _size, const double& _loops = 0.0, const double& _obstacles = 0.0, const size_t& _seed = null_v) {

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

            // Attempt to allocate the desired amount of space in memory.
            const auto product = chdr::utils::product<size_t>(_size);

            // TODO: Ensure that product does not overflow!

            std::vector<bool> result;

            try {

                std::pair<coord_t, size_t> farthest { _start, 0U };

                /*
                 * Check that the provided dimensionality of the maze is correct.
                 * The elements of _size must be at least 1.
                 */
                if (valid_dimensionality(_size)) {

                    const auto seed = _seed == null_v ? static_cast<size_t>(time(nullptr)) : _seed;
                    rng_engine_t rng(seed);

                    debug::log("\tBacktracking Algorithm \t(Seed " + std::to_string(seed) + ")");

                    // Carve a maze using the recursive backtracking algorithm:
                    result.resize(product, WALL);
                    carve_from(_start, farthest, _size, result, rng);

                    if (_loops > 0.0 || _obstacles > 0.0) {

                        // Randomly knock down walls if the maze is meant to contain loops:
                        for (size_t i = 1U; i < result.size(); ++i) {

                            const auto c = chdr::utils::to_nd(i, _size);

                            if (is_link(c) && !is_edge(c, _size)) {

                                if (const auto obstacle_chance = static_cast<double>(rng()) / static_cast<double>(rng_engine_t::max());
                                    obstacle_chance < _obstacles
                                ) {
                                    result[i] = WALL;
                                }
                                else if (const auto loop_chance = static_cast<double>(rng()) / static_cast<double>(rng_engine_t::max());
                                    loop_chance < _loops
                                ) {
                                    result[i] = PATH;
                                }
                            }
                        }
                    }
                }
                else {
                    debug::log("\tBacktracking Algorithm \tINVALID DIMENSIONALITY", error);
                }

                _end = farthest.first;
            }
            catch (const std::exception& e) {
                debug::log(e);

                _end = _start;
            }

            return result;
        }

    };

} //test::generator::utils

#endif //TEST_BACKTRACKING_HPP