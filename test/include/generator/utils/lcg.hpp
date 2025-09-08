/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef TEST_LCG_HPP
#define TEST_LCG_HPP

#include <cstdint>

namespace test::generator::utils {

    template <typename T = uint32_t>
    struct [[maybe_unused]] lcg {

        static_assert(std::is_integral_v<T>, "Template parameter T must be an integral type.");
        static_assert(std::is_unsigned_v<T>, "Template parameter T must be unsigned.");

        using result_type = std::make_unsigned_t<T>;

        result_type state;

        static constexpr result_type multiplier = sizeof(T) == 4U ?
            static_cast<result_type>(1664525U) :                    // ranqd1
            static_cast<result_type>(6364136223846793005ULL);       // MMIX

        static constexpr result_type increment = sizeof(T) == 4U ?
            static_cast<result_type>(1013904223U) :                 // ranqd1
            static_cast<result_type>(1442695040888963407ULL);       // MMIX

        static constexpr result_type modulus = sizeof(T) == 4U ?
            static_cast<result_type>(1U   << 31U) :                 // ranqd1
            static_cast<result_type>(1ULL << 63ULL);                // MMIX

        [[maybe_unused]] constexpr explicit lcg(result_type _seed = 0) noexcept :
            state(_seed) {}

        [[maybe_unused]] HOT constexpr void seed(result_type _seed) noexcept {
            state = _seed;
        }

        [[maybe_unused]] HOT constexpr result_type operator()() noexcept {
            state = (multiplier * state + increment) % modulus;
            return state;
        }

        [[maybe_unused]] HOT static constexpr result_type min() noexcept { return 0; }

        [[maybe_unused]] HOT static constexpr result_type max() noexcept { return modulus - 1; }
    };

} //test::generator::utils

#endif //TEST_LCG_HPP