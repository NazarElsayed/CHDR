#ifndef CHDR_HEURISTICS_HPP
#define CHDR_HEURISTICS_HPP

#include <Debug.hpp>

#include "types/Coord.hpp"

#ifdef __SSE2__

#include <emmintrin.h> // For SSE2 operations.

#endif // __SSE2__

namespace CHDR {

    template <const size_t Kd, typename Ts>
	struct Heuristics {

        using coord_t = Coord<size_t, Kd>;

	private:

        struct SIMDExtensions {

            struct Uint32 {

#ifdef __SSE2__

            [[nodiscard]] static constexpr Ts Sub_SSEX(const __m128i& _regA, const __m128i& _regB) {

                // Perform A - B:
                const auto sub = _mm_sub_epi32(_regA, _regB);

                __m128i resultOut{};
                _mm_store_si128(&resultOut, sub);

                const auto* const output = reinterpret_cast<size_t*>(&resultOut);
                return static_cast<Ts>(output[0U] + output[1U] + output[2U] + output[3U]);
            }

#endif //__SSE2__

            };

            struct Uint64 {

#ifdef __SSE2__

                [[nodiscard]] static constexpr Ts Sub_SSEX(const __m128i& _regA, const __m128i& _regB) {

#ifdef __SSE4_2__
                    const auto sub = _mm_sub_epi64(_regA, _regB);
#else // ifndef __SSE4_2__

                    const auto   notB = _mm_xor_si128(_regB, _mm_set1_epi64x(-1)); // bitwise not
                    const auto minusB = _mm_add_epi64(notB, _mm_set1_epi64x( 1)); // add 1

                    // Perform A - B by computing A + (-B):
                    const auto sub = _mm_add_epi64(_regA, minusB);
#endif // __SSE4_2__

                    __m128i resultOut{};
                    _mm_store_si128(&resultOut, sub);

                    const auto* const output = reinterpret_cast<size_t*>(&resultOut);
                    return static_cast<Ts>(output[0U] + output[1U]);
                }

#endif // __SSE2__

            };

            struct Float {


            };

        };

	public:

#pragma region Heuristics

        /**
         * @brief Computes the Euclidean distance between two nodes.
         *
         * @tparam Kd The number of dimensions of the nodes.
         * @param _A The first node.
         * @param _B The second node.
         * @return The Euclidean distance between _A and _B.
         */
        [[nodiscard]] static constexpr auto EuclideanDistance(const coord_t& _A, const coord_t& _B) {
            return sqrtf(SqrEuclideanDistance(_A, _B));
        }

        /*
         * @brief Computes the squared Euclidean distance between two nodes.
         *
         * @tparam Kd The number of dimensions of the nodes.
         * @param _A The first node.
         * @param _B The second node.
         * @return The squared Euclidean distance between _A and _B.
         */
        [[nodiscard]] static constexpr auto SqrEuclideanDistance(const coord_t& _A, const coord_t& _B) {

            static_assert(Kd >= 0, "Kd must be more than or equal to 0");

            Ts result(0);

            if constexpr (Kd > 0U) {

                // Non-SIMD fallback:
                for (size_t i = 0U; i < Kd; ++i) {
                    const auto val = _B[i] - _A[i];
                    result += val * val;
                }
            }

            return result;
        }

        /**
          * @brief Calculate the Manhattan distance between two nodes.
          *
          * @tparam Kd The number of dimensions of the nodes.
          * @param _A The first node.
          * @param _B The second node.
          * @return The Manhattan distance between _A and _B.
          */
        [[nodiscard]] static constexpr auto ManhattanDistance(const coord_t& _A, const coord_t& _B) {

            static_assert(Kd >= 0, "Kd must be more than or equal to 0");

            Ts result(0);

            constexpr bool is_32bit((sizeof(size_t) * 8U) == 32U);
            constexpr bool is_64bit((sizeof(size_t) * 8U) == 64U);

            if constexpr (Kd > 0U) {

                if constexpr (Kd == 1U) {
                    result = _B[0U] - _A[0U];
                }

                /* TODO: Add SIMD for other instructions sets than SSE2:
                 *
                 *  MMX: MMX registers are 64 bits wide.
                 *  SSE: Streaming SIMD Extensions (SSE) registers are 128 bits wide. [DONE]
                 *  AVX: Advanced Vector Extensions (AVX) registers are 256 bits wide.
                 *  AVX-512: Advanced Vector Extensions 512 (AVX-512) registers are 512 bits wide.
                 */
#ifdef __SSE2__
                else if constexpr (is_32bit) {

                    if constexpr (Kd == 3U) {

#ifdef __GNUC__
                        _mm_prefetch((const char*)&_A[8U], _MM_HINT_T0);
                        _mm_prefetch((const char*)&_B[8U], _MM_HINT_T0);
#endif // __GNUC__

                        result += SIMDExtensions::Uint32::Sub_SSEX(
                            _mm_set_epi32(0, _A[0U], _A[1U], _A[2U]),
                            _mm_set_epi32(0, _B[0U], _B[1U], _B[2U])
                        );
                    }
                    if constexpr (Kd == 4U) {

                        result = SIMDExtensions::Uint32::Sub_SSEX(
                            _mm_load_si128(reinterpret_cast<__m128i const*>(&_A)),
                            _mm_load_si128(reinterpret_cast<__m128i const*>(&_B))
                        );
                    }
                    else {

                        constexpr auto r = Kd % 4U;

                        size_t i;
                        for (i = 0U; i < (Kd - r); i += 4U) {

#ifdef __GNUC__
                            _mm_prefetch((const char*)&_A[8U], _MM_HINT_T0);
                            _mm_prefetch((const char*)&_B[8U], _MM_HINT_T0);
#endif // __GNUC__

                            result += SIMDExtensions::Uint32::Sub_SSEX(
                                _mm_set_epi32(_A[i], _A[i + 1U], _A[i + 2U], _A[i + 3U]),
                                _mm_set_epi32(_B[i], _B[i + 1U], _B[i + 2U], _B[i + 3U])
                            );
                        }

                        for (; i < Kd; ++i) {
                            result += _B[i] - _A[i];
                        }
                    }
                }
                else if constexpr (is_64bit) {

                    if constexpr (Kd == 2U) {

#ifdef __GNUC__
                        _mm_prefetch((const char*)&_A[8U], _MM_HINT_T0);
                        _mm_prefetch((const char*)&_B[8U], _MM_HINT_T0);
#endif // __GNUC__

                        result = SIMDExtensions::Uint64::Sub_SSEX(
                            _mm_load_si128(reinterpret_cast<__m128i const*>(&_A)),
                            _mm_load_si128(reinterpret_cast<__m128i const*>(&_B))
                        );
                    }
                    else {

                        constexpr auto r = Kd % 2U;

                        size_t i;
                        for (i = 0U; i < (Kd - r); i += 2U) {

#ifdef __GNUC__
                            _mm_prefetch((const char*)&_A[8U], _MM_HINT_T0);
                            _mm_prefetch((const char*)&_B[8U], _MM_HINT_T0);
#endif // __GNUC__

                            result += SIMDExtensions::Uint64::Sub_SSEX(
                                _mm_set_epi64x(_A[i], _A[i + 1U]),
                                _mm_set_epi64x(_B[i], _B[i + 1U])
                            );
                        }

                        if constexpr (r != 0U) {
                            result += _B[i] - _A[i];
                        }
                    }
                }
#endif // __SSE2__
                else {

                    // Non-SIMD fallback:
                    for (size_t i = 0U; i < Kd; ++i) {
                        result += _B[i] - _A[i];
                    }
                }
            }

            return result;
        }

#pragma endregion Heuristics

	};

} // CHDR

#endif //CHDR_UTILS_HPP