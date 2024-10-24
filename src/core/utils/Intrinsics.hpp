#ifndef CHDR_INTRINSICS_HPP
#define CHDR_INTRINSICS_HPP

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

#endif //CHDR_INTRINSICS_HPP