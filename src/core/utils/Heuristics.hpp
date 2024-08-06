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

            struct Uint8 {

#ifdef __SSE2__

                [[nodiscard]] static constexpr auto Sub_SSEX(const __m128i& _regA, const __m128i& _regB) {

                    // Perform A - B:
                    const auto sub = _mm_sub_epi8(_regA, _regB);

                    __m128i resultOut{};
                    _mm_store_si128(&resultOut, sub);
                    const auto* const output = reinterpret_cast<uint16_t*>(&resultOut);

                    return output[ 0U] + output[ 1U] + output[ 2U] + output[ 3U] +
                           output[ 4U] + output[ 5U] + output[ 6U] + output[ 7U] +
                           output[ 8U] + output[ 9U] + output[10U] + output[11U] +
                           output[12U] + output[13U] + output[14U] + output[15U];
                }

#endif //__SSE2__

            };

            struct Uint16 {

#ifdef __SSE2__

                [[nodiscard]] static constexpr auto Sub_SSEX(const __m128i& _regA, const __m128i& _regB) {

                    // Perform A - B:
                    const auto sub = _mm_sub_epi16(_regA, _regB);

                    __m128i resultOut{};
                    _mm_store_si128(&resultOut, sub);
                    const auto* const output = reinterpret_cast<uint16_t*>(&resultOut);

                    return output[0U] + output[1U] + output[2U] + output[3U] +
                           output[4U] + output[5U] + output[6U] + output[7U];
                }

#endif //__SSE2__

            };

            struct Uint32 {

#ifdef __SSE2__

                [[nodiscard]] static constexpr auto Sub_SSEX(const __m128i& _regA, const __m128i& _regB) {

                    // Perform A - B:
                    const auto sub = _mm_sub_epi32(_regA, _regB);

                    __m128i resultOut{};
                    _mm_store_si128(&resultOut, sub);
                    const auto* const output = reinterpret_cast<uint32_t*>(&resultOut);

                    return output[0U] + output[1U] + output[2U] + output[3U];
                }

#endif //__SSE2__

            };

            struct Uint64 {

#ifdef __SSE2__

                [[nodiscard]] static constexpr auto Sub_SSEX(const __m128i& _regA, const __m128i& _regB) {

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
                    const auto* const output = reinterpret_cast<uint64_t*>(&resultOut);

                    return output[0U] + output[1U];
                }

#endif // __SSE2__

            };

            struct Float32 {

#ifdef __SSE2__

                [[nodiscard]] static constexpr auto Sub_SSEX(const __m128& _regA, const __m128& _regB) {

                    const auto sub = _mm_sub_ps(_regA, _regB);

                    float output[4U];
                    _mm_storeu_ps(output, sub);

                    return output[0U] + output[1U] + output[2U] + output[3U];
                }

#endif // __SSE2__

            };

            struct Float64 {

#ifdef __SSE2__

                [[nodiscard]] static constexpr auto Sub_SSEX(const __m128d& _regA, const __m128d& _regB) {

                    const auto sub = _mm_sub_pd(_regA, _regB);

                    double output[4U];
                    _mm_storeu_pd(output, sub);

                    return output[0U] + output[1U];
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

            static_assert(Kd >= 0U, "Kd must be more than or equal to 0");

            Ts result(0);

            if constexpr (Kd > 0U) {

                if constexpr (Kd == 1U) {
                    // No need for SIMD if there's only one element.
                    result = static_cast<Ts>(_B[0U] - _A[0U]);
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

                    float cr = 0.0;

                    constexpr auto cA = Utils::Convert<float>(_A);
                    constexpr auto cB = Utils::Convert<float>(_B);

                    if constexpr (Kd == 3U) {

#if defined(__GNUC__) || defined(__clang__)
                        _mm_prefetch(reinterpret_cast<const char* const>(&cA[2U]), _MM_HINT_T0);
                        _mm_prefetch(reinterpret_cast<const char* const>(&cB[2U]), _MM_HINT_T0);
#endif // __GNUC__ || __clang__

                        cr = static_cast<float>(SIMDExtensions::Float32::Sub_SSEX(
                            _mm_set_ps(0, cA[0U], cA[1U], cA[2U]),
                            _mm_set_ps(0, cB[0U], cB[1U], cB[2U])
                        ));
                    }
                    if constexpr (Kd == 4U) {

#if defined(__GNUC__) || defined(__clang__)
                        _mm_prefetch(reinterpret_cast<const char* const>(&cA[2U]), _MM_HINT_T0);
                        _mm_prefetch(reinterpret_cast<const char* const>(&cB[2U]), _MM_HINT_T0);
#endif // __GNUC__ || __clang__

                        cr = static_cast<float>(SIMDExtensions::Float32::Sub_SSEX(
                            _mm_load_ps(&cA),
                            _mm_load_ps(&cB)
                        ));
                    }
                    else {

                        constexpr auto r = Kd % 4U;

                        size_t j;
                        for (j = 0U; j < (Kd - r); j += 4U) {

#if defined(__GNUC__) || defined(__clang__)
                            _mm_prefetch(reinterpret_cast<const char* const>(&cA[2U]), _MM_HINT_T0);
                            _mm_prefetch(reinterpret_cast<const char* const>(&cB[2U]), _MM_HINT_T0);
#endif // __GNUC__ || __clang__

                            const auto val = SIMDExtensions::Float32::Sub_SSEX(
                                _mm_set_ps(cA[j], cA[j + 1U], cA[j + 2U], cA[j + 3U]),
                                _mm_set_ps(cB[j], cB[j + 1U], cB[j + 2U], cB[j + 3U])
                            );

                            cr += static_cast<float>(val * val);
                        }

                        for (; j < Kd; ++j) {
                            const auto val = cB[j] - cA[j];
                            cr += static_cast<float>(val * val);
                        }
                    }

                    result = static_cast<Ts>(cr);
                }
                else if constexpr (std::is_same_v<Ts, double> || std::is_same_v<Ts, uint64_t> || std::is_same_v<Ts, int64_t>) {

                    double cr = 0.0;

                    constexpr auto cA = Utils::Convert<double>(_A);
                    constexpr auto cB = Utils::Convert<double>(_B);

                    if constexpr (Kd == 2U) {

#if defined(__GNUC__) || defined(__clang__)
                        _mm_prefetch(reinterpret_cast<const char* const>(&cA[2U]), _MM_HINT_T0);
                        _mm_prefetch(reinterpret_cast<const char* const>(&cB[2U]), _MM_HINT_T0);
#endif // __GNUC__ || __clang__

                        cr = static_cast<double>(SIMDExtensions::Float64::Sub_SSEX(
                            _mm_load_pd(&cA),
                            _mm_load_pd(&cB)
                        ));
                    }
                    else {

                        constexpr auto r = Kd % 2U;

                        size_t j;
                        for (j = 0U; j < (Kd - r); j += 2U) {

#if defined(__GNUC__) || defined(__clang__)
                            _mm_prefetch(reinterpret_cast<const char* const>(&cA[2U]), _MM_HINT_T0);
                            _mm_prefetch(reinterpret_cast<const char* const>(&cB[2U]), _MM_HINT_T0);
#endif // __GNUC__ || __clang__

                            const auto val = SIMDExtensions::Float64::Sub_SSEX(
                                _mm_set_pd(cA[j], cA[j + 1U]),
                                _mm_set_pd(cB[j], cB[j + 1U])
                            );

                            cr += static_cast<double>(val * val);
                        }

                        for (; j < Kd; ++j) {
                            const auto val = cB[j] - cA[j];
                            cr += static_cast<double>(cB[j] - cA[j]);
                        }
                    }

                    result = static_cast<Ts>(cr);
                }
#endif // __SSE2__

                else {

                    // Non-SIMD fallback:
                    for (size_t i = 0U; i < Kd; ++i) {
                        const auto val = _B[i] - _A[i];
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
          * @param _A The first node.
          * @param _B The second node.
          * @return The Manhattan distance between _A and _B.
          */
        [[nodiscard]] static constexpr auto ManhattanDistance(const coord_t& _A, const coord_t& _B) {

            static_assert(Kd >= 0U, "Kd must be more than or equal to 0");

            Ts result(0);

            if constexpr (Kd > 0U) {

                if constexpr (Kd == 1U) {
                    // No need for SIMD if there's only one element.
                    result = static_cast<Ts>(_B[0U] - _A[0U]);
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

                    constexpr auto cA = Utils::Convert<uint32_t>(_A);
                    constexpr auto cB = Utils::Convert<uint32_t>(_B);

                    if constexpr (Kd == 3U) {

#if defined(__GNUC__) || defined(__clang__)
                        _mm_prefetch(reinterpret_cast<const char* const>(&cA[2U]), _MM_HINT_T0);
                        _mm_prefetch(reinterpret_cast<const char* const>(&cB[2U]), _MM_HINT_T0);
#endif // __GNUC__ || __clang__

                        result = static_cast<Ts>(SIMDExtensions::Uint32::Sub_SSEX(
                            _mm_set_epi32(0, cA[0U], cA[1U], cA[2U]),
                            _mm_set_epi32(0, cB[0U], cB[1U], cB[2U])
                        ));
                    }
                    if constexpr (Kd == 4U) {

#if defined(__GNUC__) || defined(__clang__)
                        _mm_prefetch(reinterpret_cast<const char* const>(&cA[2U]), _MM_HINT_T0);
                        _mm_prefetch(reinterpret_cast<const char* const>(&cB[2U]), _MM_HINT_T0);
#endif // __GNUC__ || __clang__

                        result = static_cast<Ts>(SIMDExtensions::Uint32::Sub_SSEX(
                            _mm_load_si128(reinterpret_cast<__m128i const*>(&cA)),
                            _mm_load_si128(reinterpret_cast<__m128i const*>(&cB))
                        ));
                    }
                    else {

                        constexpr auto r = Kd % 4U;

                        size_t j;
                        for (j = 0U; j < (Kd - r); j += 4U) {

#if defined(__GNUC__) || defined(__clang__)
                            _mm_prefetch(reinterpret_cast<const char* const>(&cA[2U]), _MM_HINT_T0);
                            _mm_prefetch(reinterpret_cast<const char* const>(&cB[2U]), _MM_HINT_T0);
#endif // __GNUC__ || __clang__

                            result += static_cast<Ts>(SIMDExtensions::Uint32::Sub_SSEX(
                                _mm_set_epi32(cA[j], cA[j + 1U], cA[j + 2U], cA[j + 3U]),
                                _mm_set_epi32(cB[j], cB[j + 1U], cB[j + 2U], cB[j + 3U])
                            ));
                        }

                        for (; j < Kd; ++j) {
                            result += static_cast<Ts>(cB[j] - cA[j]);
                        }
                    }
                }
                else if constexpr (std::is_same_v<Ts, uint64_t> || std::is_same_v<Ts, int64_t> || std::is_same_v<Ts, double>) {

                    constexpr auto cA = Utils::Convert<uint64_t>(_A);
                    constexpr auto cB = Utils::Convert<uint64_t>(_B);

                    if constexpr (Kd == 2U) {

#if defined(__GNUC__) || defined(__clang__)
                        _mm_prefetch(reinterpret_cast<const char* const>(&cA[2U]), _MM_HINT_T0);
                        _mm_prefetch(reinterpret_cast<const char* const>(&cB[2U]), _MM_HINT_T0);
#endif // __GNUC__ || __clang__

                        result = static_cast<Ts>(SIMDExtensions::Uint64::Sub_SSEX(
                            _mm_load_si128(reinterpret_cast<__m128i const*>(&cA)),
                            _mm_load_si128(reinterpret_cast<__m128i const*>(&cB))
                        ));
                    }
                    else {

                        constexpr auto r = Kd % 2U;

                        size_t j;
                        for (j = 0U; j < (Kd - r); j += 2U) {

#if defined(__GNUC__) || defined(__clang__)
                            _mm_prefetch(reinterpret_cast<const char* const>(&cA[2U]), _MM_HINT_T0);
                            _mm_prefetch(reinterpret_cast<const char* const>(&cB[2U]), _MM_HINT_T0);
#endif // __GNUC__ || __clang__

                            result += static_cast<Ts>(SIMDExtensions::Uint64::Sub_SSEX(
                                _mm_set_epi64x(cA[j], cA[j + 1U]),
                                _mm_set_epi64x(cB[j], cB[j + 1U])
                            ));
                        }

                        for (; j < Kd; ++j) {
                            result += static_cast<Ts>(cB[j] - cA[j]);
                        }
                    }
                }
#endif // __SSE2__

                else {

                    // Non-SIMD fallback:
                    for (size_t i = 0U; i < Kd; ++i) {
                        result += static_cast<Ts>(_B[i] - _A[i]);
                    }
                }
            }

            return result;
        }

#pragma endregion Heuristics

	};

} // CHDR

#endif //CHDR_UTILS_HPP