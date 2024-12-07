/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_SIMDEXTENSIONS_HPP
#define CHDR_SIMDEXTENSIONS_HPP

#include <cstring>

#include "intrinsics.hpp"

namespace chdr {

    struct simd_extensions {

        struct [[maybe_unused]] uint8 {

    #ifdef __SSE2__

            [[maybe_unused, nodiscard]] static auto abs_sub_128_v(const __m128i& _regA, const __m128i& _regB) {

                // Perform A - B:
                const auto sub = _mm_sub_epi8(_regA, _regB);                        // NOLINT(*-simd-intrinsics)

                // Horizontal add:
                const __m128i sum1     = _mm_sad_epu8(sub, _mm_setzero_si128());
                const __m128i sum2     = _mm_shuffle_epi32(sum1, _MM_SHUFFLE(2, 3, 0, 1));
                const __m128i finalSum = _mm_add_epi16(sum1, sum2);                       // NOLINT(*-simd-intrinsics)

                uint16_t resultOut;
                memcpy(&resultOut, &finalSum, sizeof(resultOut)); // Copy first 2 bytes.

                return resultOut;
            }

    #endif //__SSE2__

        };

        struct [[maybe_unused]] uint16 {

    #ifdef __SSE2__

            [[maybe_unused, nodiscard]] static auto abs_sub_128_v(const __m128i& _regA, const __m128i& _regB) {

                // Perform A - B:
                const auto sub = _mm_sub_epi16(_regA, _regB);                       // NOLINT(*-simd-intrinsics)

                // Horizontal add:
    #ifdef __SSSE3__
                const __m128i temp1 = _mm_hadd_epi16(sub, _mm_setzero_si128());
                const __m128i temp2 = _mm_hadd_epi16(temp1, _mm_setzero_si128());
                const __m128i finalSum = _mm_hadd_epi16(temp2, _mm_setzero_si128());

                uint32_t resultOut;
                memcpy(&resultOut, &finalSum, sizeof(resultOut));  // Copy first 2 bytes.
    #else // ifndef __SSSE3__
                // First step: 16-bit pairs -> 32-bit pairs
                const __m128i zero = _mm_setzero_si128();
                const __m128i lo = _mm_unpacklo_epi16(sub, zero); // Zero extend lower 4 values.
                const __m128i hi = _mm_unpackhi_epi16(sub, zero); // Zero extend higher 4 values.

                const __m128i sum = _mm_add_epi32(lo, hi); // Add lower 4 and higher 4 values.
                const __m128i sum2 = _mm_add_epi32(sum, _mm_srli_si128(sum, 8));
                const __m128i sum3 = _mm_add_epi32(sum2, _mm_srli_si128(sum2, 4));

                // Extract the lower 32 bits, which is the sum of 16-bit integers.
                auto resultOut = static_cast<uint32_t>(_mm_cvtsi128_si32(sum3));
    #endif // __SSSE3__

                return resultOut;
            }

    #endif //__SSE2__

        };

        struct [[maybe_unused]] uint32 {

    #ifdef __SSE2__

            [[maybe_unused, nodiscard]] static auto abs_128_v(__m128i& _value) {

                // Get absolute value to prevent underflow

                __m128i result;

    #ifdef __AVX2__
                result = _mm_abs_epi32(_value);
    #else // #ifndef __AVX2__

                // Implement it manually for older CPUs
                const auto negative_mask = _mm_cmplt_epi32(_mm_set1_epi32(0), _value); // mask = sub < 0 ? 0xffffffff : 0
                const auto tmp = _mm_sub_epi32(_value, _mm_and_si128(negative_mask, _mm_set1_epi32(1))); // if sub is negative, increase its absolute value by 1

                result = _mm_xor_si128(tmp, negative_mask); // if sub is negative, flip all its bits (turns it into a positive number)
    #endif // __AVX2__

                return result;
            }

            [[maybe_unused, nodiscard]] static auto abs_sub_128_v(const __m128i& _regA, const __m128i& _regB) {

                // Perform A - B:
                const auto sub = _mm_sub_epi32(_regA, _regB);                       // NOLINT(*-simd-intrinsics)

                __m128i resultOut{};

                // Horizontal add:
    #ifdef __SSSE3__
                __m128i hadd = _mm_hadd_epi32(sub, sub);
                hadd = _mm_hadd_epi32(hadd, hadd);

                _mm_store_si128(&resultOut, hadd);
    #else // #ifndef __SSSE3__

                // Shift and add:
                __m128i high64 = _mm_shuffle_epi32(sub, _MM_SHUFFLE(1, 0, 3, 2));
                __m128i sums = _mm_add_epi32(sub, high64);

                // Move high sums to lower bits:
                high64 = _mm_shuffle_epi32(sums, _MM_SHUFFLE(0, 0, 3, 2));
                sums = _mm_add_epi32(sums, high64);

                _mm_store_si128(&resultOut, sums);
    #endif // __SSSE3__

                return reinterpret_cast<uint32_t*>(&resultOut)[0U];
            }

    #endif // __SSE2__

        };

        struct [[maybe_unused]] uint64 {

    #ifdef __SSE2__

    #ifdef __SSE4_1__

            [[maybe_unused, nodiscard]] static auto abs_128_v(__m128i& _value) {

                // Get absolute value to prevent underflow

                __m128i result;

    #ifdef __AVX512F__
                result = _mm_abs_epi64(_value);
    #elif defined(__AVX2__)
                const __m128i sign = _mm_cmpgt_epi64(_mm_set1_epi64x(0), _value);
                result = _mm_sub_epi64(_mm_xor_si128(_value,sign), sign);           // NOLINT(*-simd-intrinsics)
    #else // #ifndef __AVX2__

                // Implement it manually for older CPUs
                const auto negative_mask = _mm_cmpgt_epi64(_mm_set1_epi64x(0), _value); // mask = sub < 0 ? 0xffffffffffffffff : 0
                const auto tmp = _mm_sub_epi64(_value, _mm_and_si128(negative_mask, _mm_set1_epi64x(1))); // if sub is negative, increase its absolute value by 1

                result = _mm_xor_si128(tmp, negative_mask); // if sub is negative, flip all its bits (turns it into a positive number)
    #endif // __AVX512F__

                return result;
            }

    #endif // __SSE4_1__

            [[maybe_unused, nodiscard]] static auto abs_sub_128_v(const __m128i& _regA, const __m128i& _regB) {

                uint64_t result;

    #ifdef __SSE4_2__
                auto sub = _mm_sub_epi64(_regA, _regB);                             // NOLINT(*-simd-intrinsics)
                sub = abs_128_v(sub);

                // Unpack into 2 halves:
                const auto high = _mm_unpackhi_epi64(sub, sub);
                const auto low = _mm_unpacklo_epi64(sub, sub);

                // Horizontal add:
                const auto sum = _mm_add_epi64(high, low);                          // NOLINT(*-simd-intrinsics)

                // Extract the lower 64-bit integer:
                result = _mm_extract_epi64(sum, 0);

    #else // #ifndef __SSE4_2__

                const auto   notB = _mm_xor_si128(_regB, _mm_set1_epi64x(-1)); // bitwise not
                const auto minusB = _mm_add_epi64( notB, _mm_set1_epi64x( 1)); // add 1

                // Perform A - B by computing A + (-B):
                const auto sub = _mm_add_epi64(_regA, minusB);

                __m128i resultOut{};
                _mm_store_si128(&resultOut, sub);
                const auto* const output = reinterpret_cast<int64_t*>(&resultOut);

                result = std::abs(output[0U]) + std::abs(output[1U]);
    #endif // __SSE4_2__

                return result;
            }

    #endif // __SSE2__

        };

        struct [[maybe_unused]] float32 {

    #ifdef __SSE2__

            [[maybe_unused, nodiscard]] static auto sub_128_v(const __m128& _regA, const __m128& _regB) {

                double result{};

                const auto sub = _mm_sub_ps(_regA, _regB);                          // NOLINT(*-simd-intrinsics)

                // Horizontal add:
    #ifdef __SSSE3__

                const auto hadd = _mm_hadd_ps(sub, sub);
                result = _mm_cvtss_f32(hadd);

    #else // #ifndef __SSSE3__

                float output[2U];
                _mm_storeu_ps(output, sub);

                result = output[0U] + output[1U];
    #endif // __SSSE3__

                return result;
            }

            [[maybe_unused, nodiscard]] static auto sqr_sub_128_v(const __m128& _regA, const __m128& _regB) {

                float result;

                const auto sub = _mm_sub_ps(_regA, _regB);                          // NOLINT(*-simd-intrinsics)
                const auto sqr = _mm_mul_ps(sub, sub);                              // NOLINT(*-simd-intrinsics)

                // Horizontal add:
    #ifdef __SSSE3__

                const auto hadd = _mm_hadd_ps(sqr, sqr);
                result = _mm_cvtss_f32(hadd);

    #else // #ifndef __SSSE3__

                float output[4U];
                _mm_storeu_ps(output, sub);

                result = output[0U] + output[1U] + output[2U] + output[3U];
    #endif // __SSSE3__

                return result;
            }

    #endif // __SSE2__

        };

        struct [[maybe_unused]] float64 {

    #ifdef __SSE2__

            [[maybe_unused, nodiscard]] static auto sub_128_v(const __m128d& _regA, const __m128d& _regB) {

                double result{};

                const auto sub = _mm_sub_pd(_regA, _regB);                          // NOLINT(*-simd-intrinsics)

                // Horizontal add:
    #ifdef __SSSE3__

                const auto hadd = _mm_hadd_pd(sub, sub);
                result = _mm_cvtsd_f64(hadd);

    #else // #ifndef __SSSE3__

                double output[2U];
                _mm_storeu_pd(output, sub);

                result = output[0U] + output[1U];
    #endif // __SSSE3__

                return result;
            }

            [[maybe_unused, nodiscard]] static auto sqr_sub_128_v(const __m128d& _regA, const __m128d& _regB) {

                double result{};

                const auto sub = _mm_sub_pd(_regA, _regB);                          // NOLINT(*-simd-intrinsics)
                const auto sqr = _mm_mul_pd(sub, sub);                              // NOLINT(*-simd-intrinsics)

                // Horizontal add:
    #ifdef __SSSE3__

                const auto hadd = _mm_hadd_pd(sqr, sqr);
                result = _mm_cvtsd_f64(hadd);

    #else // #ifndef __SSSE3__

                double output[2U];
                _mm_storeu_pd(output, sub);

                result = output[0U] + output[1U];
    #endif // __SSSE3__

                return result;
            }

    #endif // __SSE2__

        };
    };

} // chdr

#endif //CHDR_SIMDEXTENSIONS_HPP