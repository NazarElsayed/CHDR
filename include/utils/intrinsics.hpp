/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_INTRINSICS_HPP
#define CHDR_INTRINSICS_HPP

/**
 * @file intrinsics.hpp
 *
 * @details Provides architecture-specific intrinsics, compiler-specific macros, and other low-level utilities,
 *          ensuring consistent behavior and performance across various development environments.
 *
 * @note This file dynamically includes SIMD instructions based on their availability.
 */

/**
 * @def IVDEP
 * @brief Enables vectorised loops (platform-specific).
 * @note Defaults to no effect on unsupported platforms.
 */

/**
 * @def VECTOR_ALWAYS
 * @brief Forces vectorisation of loops (platform-specific).
 * @note Defaults to no effect on unsupported platforms.
 */

/**
 * @def LIKELY(x)
 * @brief Hints the compiler that the condition is likely to be true.
 * @note Defaults to evaluating the condition as-is.
 * @param x The condition to evaluate.
 */

/**
 * @def UNLIKELY(x)
 * @brief Hints the compiler that the condition is likely to be false.
 * @note Defaults to evaluating the condition as-is.
 * @param x The condition to evaluate.
 */

/**
 * @def PREFETCH(P, I)
 * @brief Provides a compiler hint to prefetch memory.
 * @note Defaults to no effect on unsupported platforms.
 * @param P Pointer to the memory to prefetch.
 * @param I Prefetch hint identifier (e.g., read/write locality).
 */

/**
 * @def RESTRICT
 * @brief Specifies pointer aliasing restrictions to improve optimisation.
 * @note Defaults to no effect on unsupported platforms.
 */

/**
 * @def ALWAYS_INLINE
 * @brief Forces a function to always be inlined (platform-specific).
 * @note Defaults to no effect on unsupported platforms.
 */

/**
 * @def HOT
 * @brief Marks a function as performance-critical (platform-specific).
 * @note Defaults to no effect on unsupported platforms.
 */

/**
 * @def COLD
 * @brief Marks a function as less frequently called (platform-specific).
 * @note Defaults to no effect on unsupported platforms.
 */

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

    /** @brief Enables vectorised loops (platform-specific). */
    #define IVDEP __pragma(loop(ivdep))

    /** @brief Forces vectorisation of loops (platform-specific). */
    #define VECTOR_ALWAYS

    /** @brief Hints the compiler that the condition is likely to be true. */
    #define LIKELY(x) (x)

    /** @brief Hints the compiler that the condition is likely to be false. */
    #define UNLIKELY(x) (x)

    /** @brief Provides a compiler hint to prefetch memory. */
    #define PREFETCH(P, I) ((void))

    /** @brief Specifies pointer aliasing restrictions to improve optimisation. */
    #define RESTRICT restrict

    /** @brief Forces a function to always be inlined. */
    #define ALWAYS_INLINE __forceinline

    /** @brief Marks a function as performance-critical. */
    #define HOT

    /** @brief Marks a function as less frequently called. */
    #define COLD __declspec(noinline)

#elif defined(__INTEL_COMPILER) || defined(__INTEL_LLVM_COMPILER)

    /** @brief Enables vectorised loops (platform-specific). */
    #define IVDEP _Pragma("ivdep")

    /** @brief Forces vectorisation of loops (platform-specific). */
    #define VECTOR_ALWAYS _Pragma("vector always")

    /** @brief Hints the compiler that the condition is likely to be true. */
    #define LIKELY(x) __builtin_expect(!!(x), 1)

    /** @brief Hints the compiler that the condition is likely to be false. */
    #define UNLIKELY(x) __builtin_expect(!!(x), 0)

    /** @brief Provides a compiler hint to prefetch memory. */
    #define PREFETCH(P, I) _mm_prefetch(reinterpret_cast<const char*>(P), I)

    /** @brief Specifies pointer aliasing restrictions to improve optimisation. */
    #define RESTRICT __restrict__

    /** @brief Forces a function to always be inlined. */
    #define ALWAYS_INLINE __attribute__((always_inline))

    /** @brief Marks a function as performance-critical. */
    #define HOT __attribute__((hot))

    /** @brief Marks a function as less frequently called. */
    #define COLD __attribute__((cold))

#elif defined(__clang__)

    /** @brief Enables vectorised loops (platform-specific). */
    #define IVDEP _Pragma("clang loop vectorise(enable)")

    /** @brief Forces vectorisation of loops (platform-specific). */
    #define VECTOR_ALWAYS

    /** @brief Hints the compiler that the condition is likely to be true. */
    #define LIKELY(x) __builtin_expect(!!(x), 1)

    /** @brief Hints the compiler that the condition is likely to be false. */
    #define UNLIKELY(x) __builtin_expect(!!(x), 0)

    /** @brief Provides a compiler hint to prefetch memory. */
    #define PREFETCH(P, I) _mm_prefetch(reinterpret_cast<const char*>(P), I)

    /** @brief Specifies pointer aliasing restrictions to improve optimisation. */
    #define RESTRICT __restrict__

    /** @brief Forces a function to always be inlined. */
    #define ALWAYS_INLINE __attribute__((always_inline))

    /** @brief Marks a function as performance-critical. */
    #define HOT __attribute__((hot))

    /** @brief Marks a function as less frequently called. */
    #define COLD __attribute__((cold))

#elif defined(__GNUC__)

    /** @brief Enables vectorised loops (platform-specific). */
    #define IVDEP _Pragma("GCC ivdep")

    /** @brief Forces vectorisation of loops (platform-specific). */
    #define VECTOR_ALWAYS

    /** @brief Hints the compiler that the condition is likely to be true. */
    #define LIKELY(x) __builtin_expect(!!(x), 1)

    /** @brief Hints the compiler that the condition is likely to be false. */
    #define UNLIKELY(x) __builtin_expect(!!(x), 0)

    /** @brief Provides a compiler hint to prefetch memory. */
    #define PREFETCH(P, I) _mm_prefetch(reinterpret_cast<const char*>(P), I)

    /** @brief Specifies pointer aliasing restrictions to improve optimisation. */
    #define RESTRICT __restrict__

    /** @brief Forces a function to always be inlined. */
    #define ALWAYS_INLINE __attribute__((always_inline))

    /** @brief Marks a function as performance-critical. */
    #define HOT __attribute__((hot))

    /** @brief Marks a function as less frequently called. */
    #define COLD __attribute__((cold))

#else

    /** @brief Enables vectorised loops (platform-specific). */
    #define IVDEP

    /** @brief Forces vectorisation of loops (platform-specific). */
    #define VECTOR_ALWAYS

    /** @brief Hints the compiler that the condition is likely to be true. */
    #define LIKELY(x)

    /** @brief Hints the compiler that the condition is likely to be false. */
    #define UNLIKELY(x)

    /** @brief Provides a compiler hint to prefetch memory. */
    #define PREFETCH(P, I)

    /** @brief Specifies pointer aliasing restrictions to improve optimisation. */
    #define RESTRICT

    /** @brief Forces a function to always be inlined. */
    #define ALWAYS_INLINE

    /** @brief Marks a function as performance-critical. */
    #define HOT

    /** @brief Marks a function as less frequently called. */
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
     * @brief Forces heap defragmentation and consolidation by the internal memory allocator.
     *
     * @details
     * This function attempts to trigger heap consolidation by:
     * - Allocating a block of memory (default size: 4096 bytes).
     * - Introducing a memory barrier to avoid compiler optimisations and ensure synchronisation.
     * - Freeing the allocated memory block after the barrier.
     *
     * @remarks Preprocessor directives are used to prevent the function from being optimised by various compilers.
     * @warning This function should generally be avoided in regular code and only used when necessary for testing
     * or extreme performance tuning cases.
     *
     * @param _malloc (optional) The size of the memory block to allocate (default: 4096 bytes).
     * @note Using this function may have side effects. Do not call unless you know what you are doing.
     */
    [[maybe_unused]] inline void malloc_consolidate(size_t _malloc = 4096U) {
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