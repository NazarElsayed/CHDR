/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_HEURISTICS_HPP
#define CHDR_HEURISTICS_HPP

#include "utils/utils.hpp"
#include "types/coord.hpp"

namespace chdr {

	struct heuristics {

        /**
         * @brief Computes the Euclidean distance between two nodes.
         *
         * @param _a The first node.
         * @param _b The second node.
         * @return The Euclidean distance between _a and _b.
         */
	    template <typename scalar_t, typename coord_t>
        [[maybe_unused, nodiscard]] static constexpr auto euclidean_distance(const coord_t& _a, const coord_t& _b) noexcept {
            return static_cast<scalar_t>(sqrt(sqr_euclidean_distance(_a, _b)));
        }

        /*
         * @brief Computes the squared Euclidean distance between two nodes.
         *
         * @param _a The first node.
         * @param _b The second node.
         * @return The squared Euclidean distance between _a and _b.
         */
	    template <typename scalar_t, typename coord_t>
        [[maybe_unused, nodiscard]] static constexpr auto sqr_euclidean_distance(const coord_t& _a, const coord_t& _b) noexcept {

	        constexpr auto Kd = std::tuple_size_v<std::decay_t<coord_t>>;

            scalar_t result{0};

            IVDEP
            for (size_t i = 0U; i < Kd; ++i) {

                const auto val = static_cast<scalar_t>(utils::abs(static_cast<signed>(_b[i]) - static_cast<signed>(_a[i])));
                result += val * val;
            }

            return result;
        }

        /**
          * @brief Calculate the Manhattan distance between two nodes.
          *
          * @param _a The first node.
          * @param _b The second node.
          * @return The Manhattan distance between _a and _b.
          */
	    template <typename scalar_t, typename coord_t>
        [[maybe_unused, nodiscard]] static constexpr auto manhattan_distance(const coord_t& _a, const coord_t& _b) noexcept {

	        constexpr auto Kd = std::tuple_size_v<std::decay_t<coord_t>>;

            scalar_t result{0};

            IVDEP
            for (size_t i = 0U; i < Kd; ++i) {
                result += static_cast<scalar_t>(utils::abs(static_cast<signed>(_b[i]) - static_cast<signed>(_a[i])));
            }

            return result;
        }

	};

} //chdr

#endif //CHDR_UTILS_HPP