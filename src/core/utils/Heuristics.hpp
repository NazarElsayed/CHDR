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

#include "types/Coord.hpp"

#include "SIMDExtensions.hpp"

namespace CHDR {

    template <const size_t Kd, typename Ts>
	struct Heuristics {

        using coord_t = Coord<size_t, Kd>;

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

                if (__builtin_is_constant_evaluated()) {    // COMPILE-TIME CONSTANT

                    for (size_t i = 0U; i < Kd; ++i) {

                        SIMDExtensions::Prefetch(&_a[i], _MM_HINT_T0);
                        SIMDExtensions::Prefetch(&_b[i], _MM_HINT_T0);

                        const auto val = _b[i] - _a[i];
                        result += static_cast<Ts>(val * val);
                    }
                }
                else {                                      // RUNTIME

                    // No need for SIMD if there's only one element.
                    if constexpr (Kd == 1U) {

                        SIMDExtensions::Prefetch(&_a[0U], _MM_HINT_T0);
                        SIMDExtensions::Prefetch(&_b[0U], _MM_HINT_T0);

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

                    else if constexpr (std::is_same_v<Ts,  float> || std::is_same_v<Ts, uint32_t> || std::is_same_v<Ts, int32_t>) {

                        float cr = 0.0F;

                        const auto cA = Utils::ArrayCast<float>(_a);
                        const auto cB = Utils::ArrayCast<float>(_b);

                        if constexpr (Kd == 3U) {

                            SIMDExtensions::Prefetch(&cA[2U], _MM_HINT_T0);
                            SIMDExtensions::Prefetch(&cB[2U], _MM_HINT_T0);

                            cr = SIMDExtensions::Float32::SqrSub_128v(
                                _mm_set_ps(0, cA[0U], cA[1U], cA[2U]),
                                _mm_set_ps(0, cB[0U], cB[1U], cB[2U])
                            );
                        }
                        if constexpr (Kd == 4U) {

                            SIMDExtensions::Prefetch(&cA[3U], _MM_HINT_T0);
                            SIMDExtensions::Prefetch(&cB[3U], _MM_HINT_T0);

                            cr = SIMDExtensions::Float32::Sub_128v(
                                _mm_load_ps(&cA),
                                _mm_load_ps(&cB)
                            );
                        }
                        else {

                            constexpr auto r = Kd % 4U;

                            size_t j{};
                            for (j = 0U; j < (Kd - r); j += 4U) {

                                SIMDExtensions::Prefetch(&cA[j + 3U], _MM_HINT_T0);
                                SIMDExtensions::Prefetch(&cB[j + 3U], _MM_HINT_T0);

                                cr += SIMDExtensions::Float32::Sub_128v(
                                    _mm_set_ps(cA[j], cA[j + 1U], cA[j + 2U], cA[j + 3U]),
                                    _mm_set_ps(cB[j], cB[j + 1U], cB[j + 2U], cB[j + 3U])
                                );
                            }

                            if (j - r == 1U) {

                                SIMDExtensions::Prefetch(&cA[j + 2U], _MM_HINT_T0);
                                SIMDExtensions::Prefetch(&cB[j + 2U], _MM_HINT_T0);

                                result += static_cast<Ts>(SIMDExtensions::Float32::Sub_128v(
                                    _mm_set_epi32(0, cA[j], cA[j + 1U], cA[j + 2U]),
                                    _mm_set_epi32(0, cB[j], cB[j + 1U], cB[j + 2U])
                                ));
                            }
                            else if (j - r == 2U) {

                                SIMDExtensions::Prefetch(&cA[j + 1U], _MM_HINT_T0);
                                SIMDExtensions::Prefetch(&cB[j + 1U], _MM_HINT_T0);

                                result += static_cast<Ts>(SIMDExtensions::Float32::Sub_128v(
                                    _mm_set_epi32(0, 0, cA[j], cA[j + 1U]),
                                    _mm_set_epi32(0, 0, cB[j], cB[j + 1U])
                                ));
                            }

                            for (; j < Kd; ++j) {

                                SIMDExtensions::Prefetch(&_a[j], _MM_HINT_T0);
                                SIMDExtensions::Prefetch(&_b[j], _MM_HINT_T0);

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

                            SIMDExtensions::Prefetch(&cA[1U], _MM_HINT_T0);
                            SIMDExtensions::Prefetch(&cB[1U], _MM_HINT_T0);

                            cr = SIMDExtensions::Float64::Sub_128v(
                                _mm_load_pd(&cA[0]),
                                _mm_load_pd(&cB[0])
                            );
                        }
                        else {

                            constexpr auto r = Kd % 2U;

                            size_t j{};
                            for (j = 0U; j < (Kd - r); j += 2U) {

                                SIMDExtensions::Prefetch(&cA[j + 1U], _MM_HINT_T0);
                                SIMDExtensions::Prefetch(&cB[j + 1U], _MM_HINT_T0);

                                cr += SIMDExtensions::Float64::SqrSub_128v(
                                    _mm_set_pd(cA[j], cA[j + 1U]),
                                    _mm_set_pd(cB[j], cB[j + 1U])
                                );
                            }

                            for (; j < Kd; ++j) {

                                SIMDExtensions::Prefetch(&cA[j], _MM_HINT_T0);
                                SIMDExtensions::Prefetch(&cB[j], _MM_HINT_T0);

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

                            SIMDExtensions::Prefetch(&_a[i], _MM_HINT_T0);
                            SIMDExtensions::Prefetch(&_b[i], _MM_HINT_T0);

                            const auto val = _b[i] - _a[i];
                            result += static_cast<Ts>(val * val);
                        }
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

                if (__builtin_is_constant_evaluated()) {    // COMPILE-TIME CONSTANT

                    for (size_t i = 0U; i < Kd; ++i) {

                        SIMDExtensions::Prefetch(&_a[i], _MM_HINT_T0);
                        SIMDExtensions::Prefetch(&_b[i], _MM_HINT_T0);

                        result += static_cast<Ts>(std::abs(static_cast<signed>(_b[i]) - static_cast<signed>(_a[i])));
                    }
                }
                else {                                      // RUNTIME

                    // No need for SIMD if there's only one element.
                    if constexpr (Kd == 1U) {

                        SIMDExtensions::Prefetch(&_a[0U], _MM_HINT_T0);
                        SIMDExtensions::Prefetch(&_b[0U], _MM_HINT_T0);

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

                    else if constexpr (std::is_same_v<Ts, uint32_t> || std::is_same_v<Ts, int32_t> || std::is_same_v<Ts,  float>) {

                        const auto cA = Utils::ArrayCast<uint32_t>(_a);
                        const auto cB = Utils::ArrayCast<uint32_t>(_b);

                        if constexpr (Kd == 3U) {

                            SIMDExtensions::Prefetch(&cA[2U], _MM_HINT_T0);
                            SIMDExtensions::Prefetch(&cB[2U], _MM_HINT_T0);

                            result = static_cast<Ts>(SIMDExtensions::Uint32::AbsSub_128v(
                                _mm_set_epi32(0, cA[0U], cA[1U], cA[2U]),
                                _mm_set_epi32(0, cB[0U], cB[1U], cB[2U])
                            ));
                        }
                        if constexpr (Kd == 4U) {

                            SIMDExtensions::Prefetch(&cA[3U], _MM_HINT_T0);
                            SIMDExtensions::Prefetch(&cB[3U], _MM_HINT_T0);

                            result = static_cast<Ts>(SIMDExtensions::Uint32::AbsSub_128v(
                                _mm_load_si128(reinterpret_cast<__m128i const*>(&cA)),
                                _mm_load_si128(reinterpret_cast<__m128i const*>(&cB))
                            ));
                        }
                        else {

                            constexpr auto r = Kd % 4U;

                            size_t j{};
                            for (j = 0U; j < (Kd - r); j += 4U) {

                                SIMDExtensions::Prefetch(&cA[j + 3U], _MM_HINT_T0);
                                SIMDExtensions::Prefetch(&cB[j + 3U], _MM_HINT_T0);

                                result += static_cast<Ts>(SIMDExtensions::Uint32::AbsSub_128v(
                                    _mm_set_epi32(cA[j], cA[j + 1U], cA[j + 2U], cA[j + 3U]),
                                    _mm_set_epi32(cB[j], cB[j + 1U], cB[j + 2U], cB[j + 3U])
                                ));
                            }

                            if (j - r == 1U) {

                                SIMDExtensions::Prefetch(&cA[j + 2U], _MM_HINT_T0);
                                SIMDExtensions::Prefetch(&cB[j + 2U], _MM_HINT_T0);

                                result += static_cast<Ts>(SIMDExtensions::Uint32::AbsSub_128v(
                                    _mm_set_epi32(0, cA[j], cA[j + 1U], cA[j + 2U]),
                                    _mm_set_epi32(0, cB[j], cB[j + 1U], cB[j + 2U])
                                ));
                            }
                            else if (j - r == 2U) {

                                SIMDExtensions::Prefetch(&cA[j + 1U], _MM_HINT_T0);
                                SIMDExtensions::Prefetch(&cB[j + 1U], _MM_HINT_T0);

                                result += static_cast<Ts>(SIMDExtensions::Uint32::AbsSub_128v(
                                    _mm_set_epi32(0, 0, cA[j], cA[j + 1U]),
                                    _mm_set_epi32(0, 0, cB[j], cB[j + 1U])
                                ));
                            }

                            for (; j < Kd; ++j) {

                                SIMDExtensions::Prefetch(&cA[j], _MM_HINT_T0);
                                SIMDExtensions::Prefetch(&cB[j], _MM_HINT_T0);

                                result += static_cast<Ts>(std::abs(static_cast<signed>(cB[j]) - static_cast<signed>(cA[j])));
                            }
                        }
                    }
                    else if constexpr (std::is_same_v<Ts, uint64_t> || std::is_same_v<Ts, int64_t> || std::is_same_v<Ts, double>) {

                        const auto cA = Utils::ArrayCast<uint64_t>(_a);
                        const auto cB = Utils::ArrayCast<uint64_t>(_b);

                        if constexpr (Kd == 2U) {

                            SIMDExtensions::Prefetch(&cA[1U], _MM_HINT_T0);
                            SIMDExtensions::Prefetch(&cB[1U], _MM_HINT_T0);

                            result = static_cast<Ts>(SIMDExtensions::Uint64::AbsSub_128v(
                                _mm_load_si128(reinterpret_cast<__m128i const*>(&cA)),
                                _mm_load_si128(reinterpret_cast<__m128i const*>(&cB))
                            ));
                        }
                        else {

                            constexpr auto r = Kd % 2U;

                            size_t j{};
                            for (j = 0U; j < (Kd - r); j += 2U) {

                                SIMDExtensions::Prefetch(&cA[j + 1U], _MM_HINT_T0);
                                SIMDExtensions::Prefetch(&cB[j + 1U], _MM_HINT_T0);

                                result += static_cast<Ts>(SIMDExtensions::Uint64::AbsSub_128v(
                                    _mm_set_epi64x(cA[j], cA[j + 1U]),
                                    _mm_set_epi64x(cB[j], cB[j + 1U])
                                ));
                            }

                            for (; j < Kd; ++j) {

                                SIMDExtensions::Prefetch(&cA[j], _MM_HINT_T0);
                                SIMDExtensions::Prefetch(&cB[j], _MM_HINT_T0);

                                result += static_cast<Ts>(std::abs(static_cast<signed>(cB[j]) - static_cast<signed>(cA[j])));
                            }

                        }
                    }
#endif // __SSE2__

                    else {

                        // Non-SIMD fallback:
                        for (size_t i = 0U; i < Kd; ++i) {

                            SIMDExtensions::Prefetch(&_a[i], _MM_HINT_T0);
                            SIMDExtensions::Prefetch(&_b[i], _MM_HINT_T0);

                            result += static_cast<Ts>(std::abs(static_cast<signed>(_b[i]) - static_cast<signed>(_a[i])));
                        }
                    }
                }
            }

            return result;
        }

	};

} // CHDR

#endif //CHDR_UTILS_HPP