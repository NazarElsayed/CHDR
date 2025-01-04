/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_HEURISTICS_HPP
#define CHDR_HEURISTICS_HPP

#include <cmath>

#include "types/coord.hpp"

namespace chdr {

    template<size_t Kd, typename scalar_t>
	struct heuristics {

        using coord_t = coord<size_t, Kd>;

        /**
         * @brief Computes the Euclidean distance between two nodes.
         *
         * @tparam Kd The number of dimensions of the nodes.
         * @param _a The first node.
         * @param _b The second node.
         * @return The Euclidean distance between _A and _B.
         */
        [[maybe_unused, nodiscard]] static constexpr auto euclidean_distance(const coord_t& _a, const coord_t& _b) noexcept {
            return static_cast<scalar_t>(std::sqrt(sqr_euclidean_distance(_a, _b)));
        }

        /*
         * @brief Computes the squared Euclidean distance between two nodes.
         *
         * @tparam Kd The number of dimensions of the nodes.
         * @param _A The first node.
         * @param _B The second node.
         * @return The squared Euclidean distance between _A and _B.
         */
        [[maybe_unused, nodiscard]] static constexpr auto sqr_euclidean_distance(const coord_t& _a, const coord_t& _b) noexcept {

            // ReSharper disable once CppUnsignedZeroComparison
            static_assert(Kd >= 0U, "Kd must be more than or equal to 0");

            scalar_t result{0};

            IVDEP
            VECTOR_ALWAYS
            for (size_t i = 0U; i < Kd; ++i) {

                const auto val = _b[i] - _a[i];
                result += static_cast<scalar_t>(val * val);
            }

            return result;
        }

        /**
          * @brief Calculate the Manhattan distance between two nodes.
          *
          * @tparam Kd The number of dimensions of the nodes.
          * @param _a The first node.
          * @param _b The second node.
          * @return The Manhattan distance between _A and _B.
          */
        [[maybe_unused, nodiscard]] static constexpr auto manhattan_distance(const coord_t& _a, const coord_t& _b) noexcept {

            // ReSharper disable once CppUnsignedZeroComparison
            static_assert(Kd >= 0U, "Kd must be more than or equal to 0");

            scalar_t result{0};

            IVDEP
            VECTOR_ALWAYS
            for (size_t i = 0U; i < Kd; ++i) {
                result += static_cast<scalar_t>(std::abs(static_cast<signed>(_b[i]) - static_cast<signed>(_a[i])));
            }

            return result;
        }

	};

}//chdr

#endif //CHDR_UTILS_HPP