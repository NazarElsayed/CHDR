#ifndef CHDR_HEURISTICS_HPP
#define CHDR_HEURISTICS_HPP

#include <Debug.hpp>

#include <cmath>
#include <cstring>

#if defined(__AVX512__)
    #include <immintrin.h>
#elif defined(__AVX2__)
    #include <immintrin.h>
#elif defined(__AVX__)
    #include <immintrin.h>
#elif defined(__SSE4_2__)
    #include <nmmintrin.h>
#elif defined(__SSE4_1__)
    #include <smmintrin.h>
#elif defined(__SSSE3__)
    #include <tmmintrin.h>
#elif defined(__SSE3__)
    #include <pmmintrin.h>
#elif defined(__SSE2__)
    #include <emmintrin.h>
#elif defined(__SSE__)
    #include <xmmintrin.h>
#elif defined(__MMX__)
    #include <mmintrin.h>
#endif

#include "types/Coord.hpp"

namespace CHDR {

    template <const size_t Kd, typename Ts>
	struct Heuristics {

        using coord_t = Coord<size_t, Kd>;

	private:

        struct SIMDExtensions {

            static constexpr void Prefetch(const void* __P, const _mm_hint __I) {

#if defined(__SSE__) && (defined(__GNUC__) || defined(__clang__))
                    _mm_prefetch(__P, __I);
#endif // __SSE__ && (__GNUC__ || __clang__ )

            }
            
            struct Uint8 {

#ifdef __SSE2__

                [[nodiscard]] static auto AbsSub_128v(const __m128i& _regA, const __m128i& _regB) {

                    // Perform A - B:
                    const auto sub = _mm_sub_epi8(_regA, _regB);                        // NOLINT(*-simd-intrinsics)

                    // Horizontal add:
                    __m128i sum1 = _mm_sad_epu8(sub, _mm_setzero_si128());
                    __m128i sum2 = _mm_shuffle_epi32(sum1, _MM_SHUFFLE(2, 3, 0, 1));
                    __m128i finalSum = _mm_add_epi16(sum1, sum2);                       // NOLINT(*-simd-intrinsics)

                    uint16_t resultOut;
                    memcpy(&resultOut, &finalSum, sizeof(resultOut)); // Copy first 2 bytes.

                    return resultOut;
                }

#endif //__SSE2__

            };

            struct Uint16 {

#ifdef __SSE2__

                [[nodiscard]] static auto AbsSub_128v(const __m128i& _regA, const __m128i& _regB) {

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

            struct Uint32 {

#ifdef __SSE2__

                [[nodiscard]] static auto Abs_128v(__m128i& _value) {

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

                [[nodiscard]] static auto AbsSub_128v(const __m128i& _regA, const __m128i& _regB) {

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

            struct Uint64 {

#ifdef __SSE2__

#ifdef __SSE4_1__

                [[nodiscard]] static auto Abs_128v(__m128i& _value) {

                    // Get absolute value to prevent underflow

                    __m128i result;

#ifdef __AVX512F__
                    result = _mm_abs_epi64(_value);
#elif defined(__AVX2__)
                    __m128i sign = _mm_cmpgt_epi64(_mm_set1_epi64x(0), _value);
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

                [[nodiscard]] static auto AbsSub_128v(const __m128i& _regA, const __m128i& _regB) {

                    uint64_t result;

#ifdef __SSE4_2__
                    auto sub = _mm_sub_epi64(_regA, _regB);                             // NOLINT(*-simd-intrinsics)
                    sub = Abs_128v(sub);

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

            struct Float32 {

#ifdef __SSE2__

                [[nodiscard]] static auto SqrSub_128v(const __m128& _regA, const __m128& _regB) {

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

            struct Float64 {

#ifdef __SSE2__

                [[nodiscard]] static constexpr auto SqrSub_128v(const __m128d& _regA, const __m128d& _regB) {

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

	public:

#pragma region Heuristics

        /**
         * @brief Computes the Euclidean distance between two nodes.
         *
         * @tparam Kd The number of dimensions of the nodes.
         * @param _a The first node.
         * @param _b The second node.
         * @return The Euclidean distance between _A and _B.
         */
        [[nodiscard]] static constexpr auto EuclideanDistance(const coord_t& _a, const coord_t& _b) {
            return static_cast<Ts>(std::sqrt(SqrEuclideanDistance(_a, _b)));
        }

        /*
         * @brief Computes the squared Euclidean distance between two nodes.
         *
         * @tparam Kd The number of dimensions of the nodes.
         * @param _A The first node.
         * @param _B The second node.
         * @return The squared Euclidean distance between _A and _B.
         */
        [[nodiscard]] static constexpr auto SqrEuclideanDistance(const coord_t& _a, const coord_t& _b) {

            static_assert(Kd >= 0U, "Kd must be more than or equal to 0");

            Ts result(0);

            if constexpr (Kd > 0U) {

                // No need for SIMD if there's only one element.
                if constexpr (Kd == 1U) {

                    SIMDExtensions::Prefetch(reinterpret_cast<const char* const>(&_a[0U]), _MM_HINT_T0);
                    SIMDExtensions::Prefetch(reinterpret_cast<const char* const>(&_b[0U]), _MM_HINT_T0);

                    const auto val = _b[0U] - _a[0U];
                    result = static_cast<Ts>(val * val);
                }

                /* TODO: Add SIMD for other instructions sets than SSE2:
                 *
                 *  MMX: MMX registers are 64 bits wide.
                 *  SSE: Streaming SIMD Extensions (SSE) registers are 128 bits wide. [DONE]
                 *  AVX: Advanced Vector Extensions (AVX) registers are 256 bits wide.
                 *  AVX-512: Advanced Vector Extensions 512 (AVX-512) registers are 512 bits wide.
                 */
#ifdef __SSE2__

                if constexpr (std::is_same_v<Ts, float> || std::is_same_v<Ts, uint32_t> || std::is_same_v<Ts, int32_t>) {

                    float cr = 0.0F;

                    const auto cA = Utils::ArrayCast<float>(_a);
                    const auto cB = Utils::ArrayCast<float>(_b);

                    if constexpr (Kd == 3U) {

                        SIMDExtensions::Prefetch(reinterpret_cast<const char* const>(&cA[2U]), _MM_HINT_T0);
                        SIMDExtensions::Prefetch(reinterpret_cast<const char* const>(&cB[2U]), _MM_HINT_T0);

                        cr = SIMDExtensions::Float32::SqrSub_128v(
                            _mm_set_ps(0, cA[0U], cA[1U], cA[2U]),
                            _mm_set_ps(0, cB[0U], cB[1U], cB[2U])
                        );
                    }
                    if constexpr (Kd == 4U) {

                        SIMDExtensions::Prefetch(reinterpret_cast<const char* const>(&cA[3U]), _MM_HINT_T0);
                        SIMDExtensions::Prefetch(reinterpret_cast<const char* const>(&cB[3U]), _MM_HINT_T0);

                        cr = SIMDExtensions::Float32::Sub_128v(
                            _mm_load_ps(&cA),
                            _mm_load_ps(&cB)
                        );
                    }
                    else {

                        constexpr auto r = Kd % 4U;

                        size_t j{};
                        for (j = 0U; j < (Kd - r); j += 4U) {

                            SIMDExtensions::Prefetch(reinterpret_cast<const char* const>(&cA[j + 3U]), _MM_HINT_T0);
                            SIMDExtensions::Prefetch(reinterpret_cast<const char* const>(&cB[j + 3U]), _MM_HINT_T0);

                            cr += SIMDExtensions::Float32::Sub_128v(
                                _mm_set_ps(cA[j], cA[j + 1U], cA[j + 2U], cA[j + 3U]),
                                _mm_set_ps(cB[j], cB[j + 1U], cB[j + 2U], cB[j + 3U])
                            );
                        }

                        if (j - r == 1U) {

                            SIMDExtensions::Prefetch(reinterpret_cast<const char* const>(&cA[j + 2U]), _MM_HINT_T0);
                            SIMDExtensions::Prefetch(reinterpret_cast<const char* const>(&cB[j + 2U]), _MM_HINT_T0);

                            result += static_cast<Ts>(SIMDExtensions::Float32::Sub_128v(
                                _mm_set_epi32(0, cA[j], cA[j + 1U], cA[j + 2U]),
                                _mm_set_epi32(0, cB[j], cB[j + 1U], cB[j + 2U])
                            ));
                        }
                        else if (j - r == 2U) {

                            SIMDExtensions::Prefetch(reinterpret_cast<const char* const>(&cA[j + 1U]), _MM_HINT_T0);
                            SIMDExtensions::Prefetch(reinterpret_cast<const char* const>(&cB[j + 1U]), _MM_HINT_T0);

                            result += static_cast<Ts>(SIMDExtensions::Float32::Sub_128v(
                                _mm_set_epi32(0, 0, cA[j], cA[j + 1U]),
                                _mm_set_epi32(0, 0, cB[j], cB[j + 1U])
                            ));
                        }

                        for (; j < Kd; ++j) {

                            SIMDExtensions::Prefetch(reinterpret_cast<const char* const>(&_a[j]), _MM_HINT_T0);
                            SIMDExtensions::Prefetch(reinterpret_cast<const char* const>(&_b[j]), _MM_HINT_T0);

                            const auto val = cB[j] - cA[j];
                            cr += static_cast<float>(val * val);
                        }
                    }

                    result = static_cast<Ts>(cr);
                }
                else if constexpr (std::is_same_v<Ts, double> || std::is_same_v<Ts, uint64_t> || std::is_same_v<Ts, int64_t>) {

                    double cr = 0.0;

                    const auto cA = Utils::ArrayCast<double>(_a);
                    const auto cB = Utils::ArrayCast<double>(_b);

                    if constexpr (Kd == 2U) {

                        SIMDExtensions::Prefetch(reinterpret_cast<const char* const>(&cA[1U]), _MM_HINT_T0);
                        SIMDExtensions::Prefetch(reinterpret_cast<const char* const>(&cB[1U]), _MM_HINT_T0);

                        cr = SIMDExtensions::Float64::Sub_128v(
                            _mm_load_pd(&cA[0]),
                            _mm_load_pd(&cB[0])
                        );
                    }
                    else {

                        constexpr auto r = Kd % 2U;

                        size_t j{};
                        for (j = 0U; j < (Kd - r); j += 2U) {

                            SIMDExtensions::Prefetch(reinterpret_cast<const char* const>(&cA[j + 1U]), _MM_HINT_T0);
                            SIMDExtensions::Prefetch(reinterpret_cast<const char* const>(&cB[j + 1U]), _MM_HINT_T0);

                            cr += SIMDExtensions::Float64::SqrSub_128v(
                                _mm_set_pd(cA[j], cA[j + 1U]),
                                _mm_set_pd(cB[j], cB[j + 1U])
                            );
                        }

                        for (; j < Kd; ++j) {

                            SIMDExtensions::Prefetch(reinterpret_cast<const char* const>(&cA[j]), _MM_HINT_T0);
                            SIMDExtensions::Prefetch(reinterpret_cast<const char* const>(&cB[j]), _MM_HINT_T0);

                            const auto val = cB[j] - cA[j];
                            cr += static_cast<double>(val * val);
                        }
                    }

                    result = static_cast<Ts>(cr);
                }
#endif // __SSE2__

                else {

                    // Non-SIMD fallback:
                    for (size_t i = 0U; i < Kd; ++i) {

                        SIMDExtensions::Prefetch(reinterpret_cast<const char* const>(&_a[i]), _MM_HINT_T0);
                        SIMDExtensions::Prefetch(reinterpret_cast<const char* const>(&_b[i]), _MM_HINT_T0);

                        const auto val = _b[i] - _a[i];
                        result += static_cast<Ts>(val * val);
                    }
                }
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
        [[nodiscard]] static constexpr auto ManhattanDistance(const coord_t& _a, const coord_t& _b) {

            static_assert(Kd >= 0U, "Kd must be more than or equal to 0");

            Ts result(0);

            if constexpr (Kd > 0U) {

                // No need for SIMD if there's only one element.
                if constexpr (Kd == 1U) {

                    SIMDExtensions::Prefetch(reinterpret_cast<const char* const>(&_a[0U]), _MM_HINT_T0);
                    SIMDExtensions::Prefetch(reinterpret_cast<const char* const>(&_b[0U]), _MM_HINT_T0);

                    result = static_cast<Ts>(std::abs(static_cast<signed>(_b[0U]) - static_cast<signed>(_a[0U])));
                }

                /* TODO: Add SIMD for other instructions sets than SSE2:
                 *
                 *  MMX: MMX registers are 64 bits wide.
                 *  SSE: Streaming SIMD Extensions (SSE) registers are 128 bits wide. [DONE]
                 *  AVX: Advanced Vector Extensions (AVX) registers are 256 bits wide.
                 *  AVX-512: Advanced Vector Extensions 512 (AVX-512) registers are 512 bits wide.
                 */
#ifdef __SSE2__

                if constexpr (std::is_same_v<Ts, uint32_t> || std::is_same_v<Ts, int32_t> || std::is_same_v<Ts, float>) {

                    const auto cA = Utils::ArrayCast<uint32_t>(_a);
                    const auto cB = Utils::ArrayCast<uint32_t>(_b);

                    if constexpr (Kd == 3U) {

                        SIMDExtensions::Prefetch(reinterpret_cast<const char* const>(&cA[2U]), _MM_HINT_T0);
                        SIMDExtensions::Prefetch(reinterpret_cast<const char* const>(&cB[2U]), _MM_HINT_T0);

                        result = static_cast<Ts>(SIMDExtensions::Uint32::AbsSub_128v(
                            _mm_set_epi32(0, cA[0U], cA[1U], cA[2U]),
                            _mm_set_epi32(0, cB[0U], cB[1U], cB[2U])
                        ));
                    }
                    if constexpr (Kd == 4U) {

                        SIMDExtensions::Prefetch(reinterpret_cast<const char* const>(&cA[3U]), _MM_HINT_T0);
                        SIMDExtensions::Prefetch(reinterpret_cast<const char* const>(&cB[3U]), _MM_HINT_T0);

                        result = static_cast<Ts>(SIMDExtensions::Uint32::AbsSub_128v(
                            _mm_load_si128(reinterpret_cast<__m128i const*>(&cA)),
                            _mm_load_si128(reinterpret_cast<__m128i const*>(&cB))
                        ));
                    }
                    else {

                        constexpr auto r = Kd % 4U;

                        size_t j{};
                        for (j = 0U; j < (Kd - r); j += 4U) {

                            SIMDExtensions::Prefetch(reinterpret_cast<const char* const>(&cA[j + 3U]), _MM_HINT_T0);
                            SIMDExtensions::Prefetch(reinterpret_cast<const char* const>(&cB[j + 3U]), _MM_HINT_T0);

                            result += static_cast<Ts>(SIMDExtensions::Uint32::AbsSub_128v(
                                _mm_set_epi32(cA[j], cA[j + 1U], cA[j + 2U], cA[j + 3U]),
                                _mm_set_epi32(cB[j], cB[j + 1U], cB[j + 2U], cB[j + 3U])
                            ));
                        }

                        if (j - r == 1U) {

                            SIMDExtensions::Prefetch(reinterpret_cast<const char* const>(&cA[j + 2U]), _MM_HINT_T0);
                            SIMDExtensions::Prefetch(reinterpret_cast<const char* const>(&cB[j + 2U]), _MM_HINT_T0);

                            result += static_cast<Ts>(SIMDExtensions::Uint32::AbsSub_128v(
                                _mm_set_epi32(0, cA[j], cA[j + 1U], cA[j + 2U]),
                                _mm_set_epi32(0, cB[j], cB[j + 1U], cB[j + 2U])
                            ));
                        }
                        else if (j - r == 2U) {

                            SIMDExtensions::Prefetch(reinterpret_cast<const char* const>(&cA[j + 1U]), _MM_HINT_T0);
                            SIMDExtensions::Prefetch(reinterpret_cast<const char* const>(&cB[j + 1U]), _MM_HINT_T0);

                            result += static_cast<Ts>(SIMDExtensions::Uint32::AbsSub_128v(
                                _mm_set_epi32(0, 0, cA[j], cA[j + 1U]),
                                _mm_set_epi32(0, 0, cB[j], cB[j + 1U])
                            ));
                        }

                        for (; j < Kd; ++j) {

                            SIMDExtensions::Prefetch(reinterpret_cast<const char* const>(&cA[j]), _MM_HINT_T0);
                            SIMDExtensions::Prefetch(reinterpret_cast<const char* const>(&cB[j]), _MM_HINT_T0);

                            result += static_cast<Ts>(std::abs(static_cast<signed>(cB[j]) - static_cast<signed>(cA[j])));
                        }
                    }
                }
                else if constexpr (std::is_same_v<Ts, uint64_t> || std::is_same_v<Ts, int64_t> || std::is_same_v<Ts, double>) {

                    const auto cA = Utils::ArrayCast<uint64_t>(_a);
                    const auto cB = Utils::ArrayCast<uint64_t>(_b);

                    if constexpr (Kd == 2U) {

                        SIMDExtensions::Prefetch(reinterpret_cast<const char* const>(&cA[1U]), _MM_HINT_T0);
                        SIMDExtensions::Prefetch(reinterpret_cast<const char* const>(&cB[1U]), _MM_HINT_T0);

                        result = static_cast<Ts>(SIMDExtensions::Uint64::AbsSub_128v(
                            _mm_load_si128(reinterpret_cast<__m128i const*>(&cA)),
                            _mm_load_si128(reinterpret_cast<__m128i const*>(&cB))
                        ));
                    }
                    else {

                        constexpr auto r = Kd % 2U;

                        size_t j{};
                        for (j = 0U; j < (Kd - r); j += 2U) {

                            SIMDExtensions::Prefetch(reinterpret_cast<const char* const>(&cA[j + 1U]), _MM_HINT_T0);
                            SIMDExtensions::Prefetch(reinterpret_cast<const char* const>(&cB[j + 1U]), _MM_HINT_T0);

                            result += static_cast<Ts>(SIMDExtensions::Uint64::AbsSub_128v(
                                _mm_set_epi64x(cA[j], cA[j + 1U]),
                                _mm_set_epi64x(cB[j], cB[j + 1U])
                            ));
                        }

                        for (; j < Kd; ++j) {

                            SIMDExtensions::Prefetch(reinterpret_cast<const char* const>(&cA[j]), _MM_HINT_T0);
                            SIMDExtensions::Prefetch(reinterpret_cast<const char* const>(&cB[j]), _MM_HINT_T0);

                            result += static_cast<Ts>(std::abs(static_cast<signed>(cB[j]) - static_cast<signed>(cA[j])));
                        }

                    }
                }
#endif // __SSE2__

                else {

                    // Non-SIMD fallback:
                    for (size_t i = 0U; i < Kd; ++i) {

                        SIMDExtensions::Prefetch(reinterpret_cast<const char* const>(&_a[i]), _MM_HINT_T0);
                        SIMDExtensions::Prefetch(reinterpret_cast<const char* const>(&_b[i]), _MM_HINT_T0);

                        result += static_cast<Ts>(std::abs(static_cast<signed>(_b[i]) - static_cast<signed>(_a[i])));
                    }
                }
            }

            return result;
        }

#pragma endregion Heuristics

	};

} // CHDR

#endif //CHDR_UTILS_HPP