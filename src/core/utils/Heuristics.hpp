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

#ifdef __SSE2__

        [[nodiscard]] static constexpr Ts simd_sub_64bit_128(const coord_t& _A, const coord_t& _B) {

            const auto regA = _mm_load_si128(reinterpret_cast<__m128i const*>(&_A));
            const auto regB = _mm_load_si128(reinterpret_cast<__m128i const*>(&_B));

#ifdef __SSE4_2__
            const auto sub = _mm_sub_epi64(regA, regB);
#else

            const auto   notB = _mm_xor_si128(regB, _mm_set1_epi64x(-1)); // bitwise not
            const auto minusB = _mm_add_epi64(notB, _mm_set1_epi64x( 1)); // add 1

            // Perform A - B by computing A + (-B):
            const auto sub = _mm_add_epi64(regA, minusB);
#endif

            __m128i resultOut{};
            _mm_store_si128(&resultOut, sub);

            const auto* const output = reinterpret_cast<size_t*>(&resultOut);
            return static_cast<Ts>(output[0U] + output[1U]);
        }

        [[nodiscard]] static constexpr Ts simd_sub_64bit_128(
            const size_t& _A0, const size_t& _A1,
            const size_t& _B0, const size_t& _B1
        ) {

            const auto regA = _mm_set_epi64x(_A0, _A1);
            const auto regB = _mm_set_epi64x(_B0, _B1);

#ifdef __SSE4_2__
            const auto sub = _mm_sub_epi64(regA, regB);
#else

            const auto   notB = _mm_xor_si128(regB, _mm_set1_epi64x(-1)); // bitwise not
            const auto minusB = _mm_add_epi64(notB, _mm_set1_epi64x( 1)); // add 1

            // Perform A - B by computing A + (-B):
            const auto sub = _mm_add_epi64(regA, minusB);
#endif

            __m128i resultOut{};
            _mm_store_si128(&resultOut, sub);

            const auto* const output = reinterpret_cast<size_t*>(&resultOut);
            return static_cast<Ts>(output[0U] + output[1U]);
        }

        [[nodiscard]] static constexpr Ts simd_sub_32_bit_128(const coord_t& _A, const coord_t& _B) {

            // Load the coordinates into SSE registers:
            const auto regA = _mm_load_si128(reinterpret_cast<__m128i const*>(&_A));
            const auto regB = _mm_load_si128(reinterpret_cast<__m128i const*>(&_B));

            // Perform A - B:
            const auto sub = _mm_sub_epi32(regA, regB);

            __m128i resultOut{};
            _mm_store_si128(&resultOut, sub);

            const auto* const output = reinterpret_cast<size_t*>(&resultOut);
            return static_cast<Ts>(output[0U] + output[1U] + output[2U] + output[3U]);
        }

        [[nodiscard]] static constexpr Ts simd_sub_32_bit_128(
            const size_t& _A0, const size_t& _A1, const size_t& _A2, const size_t& _A3,
            const size_t& _B0, const size_t& _B1, const size_t& _B2, const size_t& _B3
        ) {

            const auto regA = _mm_set_epi32(_A3, _A2, _A1, _A0); // Load as { A3, A2, A1, A0 }
            const auto regB = _mm_set_epi32(_B3, _B2, _B1, _B0); // Load as { A3, B2, B1, B0 }

            // Perform A - B:
            const auto sub = _mm_sub_epi32(regA, regB);

            __m128i resultOut{};
            _mm_store_si128(&resultOut, sub);

            const auto* const output = reinterpret_cast<size_t*>(&resultOut);
            return static_cast<Ts>(output[0U] + output[1U] + output[2U] + output[3U]);
        }

#endif //__SSE2__

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

            Ts result(0);

            for (size_t i = 0U; i < Kd; ++i) {
                const auto val = _B[i] - _A[i];
                result += val * val;
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
                 *  SSE: Streaming SIMD Extensions (SSE) registers are 128 bits wide.
                 *  AVX: Advanced Vector Extensions (AVX) registers are 256 bits wide.
                 *  AVX-512: Advanced Vector Extensions 512 (AVX-512) registers are 512 bits wide.
                 */
#ifdef __SSE2__
                else if constexpr (is_64bit) {

                    if constexpr (Kd == 2U) {

#ifdef __GNUC__
                        _mm_prefetch((const char*)&_A[8], _MM_HINT_T0);
                        _mm_prefetch((const char*)&_B[8], _MM_HINT_T0);
#endif // __GNUC__

                        result = SIMDExtensions::simd_sub_64bit_128(_A, _B);
                    }
                    else {

                        constexpr auto r = Kd % 2U;

                        size_t i;
                        for (i = 0U; i < (Kd - r); i += 2U) {

#ifdef __GNUC__
                            _mm_prefetch((const char*)&_A[8], _MM_HINT_T0);
                            _mm_prefetch((const char*)&_B[8], _MM_HINT_T0);
#endif // __GNUC__

                            result += SIMDExtensions::simd_sub_64bit_128(
                                _A[i], _A[i + 1],
                                _B[i], _B[i + 1]
                            );
                        }

                        if constexpr (r != 0U) {
                            result += _B[i] - _A[i];
                        }
                    }
                }
                else if constexpr (is_32bit) {

                    if constexpr (Kd == 3U) {

#ifdef __GNUC__
                        _mm_prefetch((const char*)&_A[8], _MM_HINT_T0);
                        _mm_prefetch((const char*)&_B[8], _MM_HINT_T0);
#endif // __GNUC__

                        result = SIMDExtensions::simd_sub_32_bit_128(
                            0U, _A[0], _A[1], _A[2],
                            0U, _B[0], _B[1], _B[2]
                        );
                    }
                    if constexpr (Kd == 4U) {
                        result = SIMDExtensions::simd_sub_32_bit_128(_A, _B);
                    }
                    else {

                        constexpr auto r = Kd % 4U;

                        size_t i;
                        for (i = 0U; i < (Kd - r); i += 4U) {

#ifdef __GNUC__
                            _mm_prefetch((const char*)&_A[8], _MM_HINT_T0);
                            _mm_prefetch((const char*)&_B[8], _MM_HINT_T0);
#endif // __GNUC__

                            result += SIMDExtensions::simd_sub_32bit_128(
                                _A[i], _A[i + 1], _A[i + 2], _A[i + 3],
                                _B[i], _B[i + 1], _B[i + 2], _B[i + 3]
                            );
                        }

                        for (; i < Kd; ++i) {
                            result += _B[i] - _A[i];
                        }
                    }
                }
#endif // __SSE2__
                else {

                    for (size_t i = 0U; i < Kd; ++i) {
                        result += _B[i] - _A[i];
                    }
                }

                return result;
            }
        }

#pragma endregion Heuristics

	};

} // CHDR

#endif //CHDR_UTILS_HPP