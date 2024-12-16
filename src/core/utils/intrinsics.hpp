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

// ReSharper disable once CppUnnamedNamespaceInHeaderFile
namespace {

#ifdef _MSC_VER
#define IVDEP __pragma(loop(ivdep))
#define VECTOR_ALWAYS
#define LIKELY(x)   (x)
#define UNLIKELY(x) (x)
#define PREFETCH(P, I) ((void)))
#define RESTRICT restrict
#elif defined(__INTEL_COMPILER)
#define IVDEP _Pragma("ivdep")
#define VECTOR_ALWAYS _Pragma("vector always")
#define LIKELY(x)   __builtin_expect(!!(x), 1)
#define UNLIKELY(x) __builtin_expect(!!(x), 0)
#define PREFETCH(P, I) _mm_prefetch(reinterpret_cast<const char*>(P), I)
#define RESTRICT __restrict__
#elif defined(__clang__)
#define IVDEP _Pragma("clang loop vectorize(enable)")
#define VECTOR_ALWAYS
#define LIKELY(x)   __builtin_expect(!!(x), 1)
#define UNLIKELY(x) __builtin_expect(!!(x), 0)
#define PREFETCH(P, I) _mm_prefetch(reinterpret_cast<const char*>(P), I)
#define RESTRICT __restrict__
#elif defined(__GNUC__)
#define IVDEP _Pragma("GCC ivdep")
#define VECTOR_ALWAYS
#define LIKELY(x)   __builtin_expect(!!(x), 1)
#define UNLIKELY(x) __builtin_expect(!!(x), 0)
#define PREFETCH(P, I) _mm_prefetch(reinterpret_cast<const char*>(P), I)
#define RESTRICT __restrict__
#else
#define IVDEP
#define VECTOR_ALWAYS
#define LIKELY(x)   (x)
#define UNLIKELY(x) (x)
#define PREFETCH(P, I) ((void)))
#define RESTRICT
#endif

#ifdef _MSC_VER
#pragma optimize("", off)
#elif defined(__INTEL_COMPILER) || defined(__ICC)
#pragma optimize("", off)
#elif defined(__clang__)
#pragma clang optimize off
#elif defined(__GNUC__)
#pragma GCC push_options
#pragma GCC optimize ("O0")
#endif

    [[maybe_unused]]
    void malloc_consolidate(const size_t& _malloc = static_cast<size_t>(1024U) * static_cast<size_t>(2U)) {

        /*
         * Code designed to purposefully trigger heap defragmentation by the internal memory allocator.
         * Please note that calling this function should generally be avoided unless for good reason.
         *
         * An explanation:
         *     1: Attempts to allocate a block of memory (by default 2KB), which is hopefully enough to force heap consolidation.
         *     2: Frees the allocated block since it is no longer needed.
         *     ~: Memory barrier between the malloc and free to discourage optimisation and force synchronisation.
         *        (cont.) Preprocessor blocks around the function to prevent optimisation by a variety of compilers.
         */

        void* tmp = malloc(_malloc);
        asm volatile("" ::: "memory");
        free(tmp);
    }

#ifdef _MSC_VER
#pragma optimize("", on)
#elif defined(__INTEL_COMPILER) || defined(__ICC)
#pragma optimize("", on)
#elif defined(__clang__)
#pragma clang optimize on
#elif defined(__GNUC__)
#pragma GCC pop_options
#endif

} //

#endif //CHDR_INTRINSICS_HPP