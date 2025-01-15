/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_HEURISTICS_HPP
#define CHDR_HEURISTICS_HPP

#include <cstddef>

#include "../utils/utils.hpp"

// ReSharper disable once CppUnusedIncludeDirective
#include "../utils/intrinsics.hpp" // NOLINT(*-include-cleaner)

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

        /**
         * @brief Computes the Chebyshev distance between two nodes.
         *
         * @param _a The first node.
         * @param _b The second node.
         * @return The Chebyshev distance between _a and _b.
         */
        template <typename scalar_t, typename coord_t>
        [[maybe_unused, nodiscard]] static constexpr auto chebyshev_distance(const coord_t& _a, const coord_t& _b) noexcept {

	        constexpr auto Kd = std::tuple_size_v<std::decay_t<coord_t>>;

	        scalar_t result{0};

	        IVDEP
	        for (size_t i = 0U; i < Kd; ++i) {

		        const auto val = static_cast<scalar_t>(utils::abs(static_cast<signed>(_b[i]) - static_cast<signed>(_a[i])));
	        	if (val > result) {
			        result = val;
		        }
	        }

	        return result;
        }

        /**
         * @brief Computes the Canberra distance between two nodes.
         *
         * @param _a The first node.
         * @param _b The second node.
         * @return The Canberra distance between _a and _b.
         */
        template <typename scalar_t, typename coord_t>
        [[maybe_unused, nodiscard]] static constexpr auto canberra_distance(const coord_t& _a, const coord_t& _b) noexcept {

	    	constexpr auto Kd = std::tuple_size_v<std::decay_t<coord_t>>;

	        scalar_t result{0};

	        IVDEP
	        for (size_t i = 0U; i < Kd; ++i) {

	        	if (const auto denominator = static_cast<scalar_t>(utils::abs(_a[i]) + utils::abs(_b[i])); denominator != 0) {
	        		const auto   numerator = static_cast<scalar_t>(utils::abs(static_cast<signed>(_b[i]) - static_cast<signed>(_a[i])));

			        result += numerator / denominator;
		        }
	        }

	        return result;
        }

        /**
         * @brief Computes the Cosine distance between two nodes.
         *
         * @param _a The first node.
         * @param _b The second node.
         * @return The Cosine distance (1 - Cosine similarity) between _a and _b.
         */
        template <typename scalar_t, typename coord_t>
        [[maybe_unused, nodiscard]] static constexpr auto cosine_distance(const coord_t& _a, const coord_t& _b) noexcept {

	    	constexpr auto Kd = std::tuple_size_v<std::decay_t<coord_t>>;

	        scalar_t dot_product {0};
	        scalar_t norm_a      {0};
	        scalar_t norm_b      {0};

	        IVDEP
	        for (size_t i = 0; i < Kd; ++i) {
	        	
		        const scalar_t val_a = static_cast<scalar_t>(_a[i]);
		        const scalar_t val_b = static_cast<scalar_t>(_b[i]);
	        	
		        dot_product += val_a * val_b;
		        norm_a      += val_a * val_a;
		        norm_b      += val_b * val_b;
	        }

	        const auto norm_product = utils::sqrt(norm_a) * utils::sqrt(norm_b);
	    	
	        return (norm_product > 0) ? (1 - (dot_product / norm_product)) : static_cast<scalar_t>(1);
        }
	};

} //chdr

#endif //CHDR_UTILS_HPP