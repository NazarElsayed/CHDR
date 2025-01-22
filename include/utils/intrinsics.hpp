/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_INTRINSICS_HPP
#define CHDR_INTRINSICS_HPP

#include <cstddef>
#include <cstdlib>

/* ReSharper disable CppUnusedIncludeDirective */
// NOLINTBEGIN(*-include-cleaner)

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

// NOLINTEND(*-include-cleaner)
/* ReSharper enable CppUnusedIncludeDirective */

namespace chdr {

#ifdef _MSC_VER
#define IVDEP __pragma(loop(ivdep))
#define VECTOR_ALWAYS
#define LIKELY(x)   (x)
#define UNLIKELY(x) (x)
#define PREFETCH(P, I) ((void)))
#define RESTRICT restrict
#define ALWAYS_INLINE __forceinline
#define HOT
#define COLD __declspec(noinline)
#elif defined(__INTEL_COMPILER) || defined(__INTEL_LLVM_COMPILER)
#define IVDEP _Pragma("ivdep")
#define VECTOR_ALWAYS _Pragma("vector always")
#define LIKELY(x)   __builtin_expect(!!(x), 1)
#define UNLIKELY(x) __builtin_expect(!!(x), 0)
#define PREFETCH(P, I) _mm_prefetch(reinterpret_cast<const char*>(P), I)
#define RESTRICT __restrict__
#define ALWAYS_INLINE __attribute__((always_inline))
#define HOT __attribute__((hot))
#define COLD __attribute__((cold))
#elif defined(__clang__)
#define IVDEP _Pragma("clang loop vectorize(enable)")
#define VECTOR_ALWAYS
#define LIKELY(x)   __builtin_expect(!!(x), 1)
#define UNLIKELY(x) __builtin_expect(!!(x), 0)
#define PREFETCH(P, I) _mm_prefetch(reinterpret_cast<const char*>(P), I)
#define RESTRICT __restrict__
#define ALWAYS_INLINE __attribute__((always_inline))
#define HOT __attribute__((hot))
#define COLD __attribute__((cold))
#elif defined(__GNUC__)
#define IVDEP _Pragma("GCC ivdep")
#define VECTOR_ALWAYS
#define LIKELY(x)   __builtin_expect(!!(x), 1)
#define UNLIKELY(x) __builtin_expect(!!(x), 0)
#define PREFETCH(P, I) _mm_prefetch(reinterpret_cast<const char*>(P), I)
#define RESTRICT __restrict__
#define ALWAYS_INLINE __attribute__((always_inline))
#define HOT __attribute__((hot))
#define COLD __attribute__((cold))
#else
#define IVDEP
#define VECTOR_ALWAYS
#define LIKELY(x)   (x)
#define UNLIKELY(x) (x)
#define PREFETCH(P, I) ((void)))
#define RESTRICT
#define ALWAYS_INLINE
#define HOT
#define COLD
#endif

#ifdef _MSC_VER
#pragma optimize("", off)
#elif defined(__INTEL_COMPILER) || defined(__INTEL_LLVM_COMPILER)
#pragma clang optimize off
#elif defined(__clang__)
#pragma clang optimize off
#elif defined(__GNUC__)
#pragma GCC push_options
#pragma GCC optimize ("O0")
#endif

    /**
     * @brief Function to purposefully trigger heap defragmentation by the internal memory allocator.
     * @note Calling this function should generally be avoided unless you know what you are doing.
     *
     * @details
     * Details:
     * - Attempts to allocate a block of memory (by default 2048 bytes), which is 'hopefully enough' to force heap consolidation.
     * - Frees the allocated block since it is no longer needed.
     * - Includes a memory barrier between the malloc and free to discourage optimisation and force synchronisation.
     * - Uses preprocessor blocks around the function to prevent optimisation by a variety of compilers.
     */
    [[maybe_unused]] inline void malloc_consolidate(size_t _malloc = 2048U) {
        void* tmp = malloc(_malloc);
        asm volatile("" ::: "memory");
        free(tmp);
    }

#ifdef _MSC_VER
#pragma optimize("", on)
#elif defined(__INTEL_COMPILER) || defined(__INTEL_LLVM_COMPILER)
#pragma clang optimize on
#elif defined(__clang__)
#pragma clang optimize on
#elif defined(__GNUC__)
#pragma GCC pop_options
#endif

} //chdr

#endif //CHDR_INTRINSICS_HPP