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
    #define PRAGMA_IVDEP __pragma(loop(ivdep))
    #define PRAGMA_VECTOR_ALWAYS
#elif defined(__INTEL_COMPILER)
    #define PRAGMA_IVDEP _Pragma("ivdep")
    #define PRAGMA_VECTOR_ALWAYS _Pragma("vector always")
#elif defined(__clang__)
    #define PRAGMA_IVDEP _Pragma("clang loop vectorize(enable)")
    #define PRAGMA_VECTOR_ALWAYS
#elif defined(__GNUC__)
    #define PRAGMA_IVDEP _Pragma("GCC ivdep")
    #define PRAGMA_VECTOR_ALWAYS
#else
    #define PRAGMA_IVDEP
    #define PRAGMA_VECTOR_ALWAYS
#endif

static constexpr void prefetch(const void* __P, const _mm_hint& __I) {

#if defined(__SSE__) && (defined(__GNUC__) || defined(__clang__))

    switch (__I) {
        case _MM_HINT_T0:  { _mm_prefetch(__P, _MM_HINT_T0 ); break; }
        case _MM_HINT_T1:  { _mm_prefetch(__P, _MM_HINT_T1 ); break; }
        case _MM_HINT_T2:  { _mm_prefetch(__P, _MM_HINT_T2 ); break; }
        case _MM_HINT_NTA: { _mm_prefetch(__P, _MM_HINT_NTA); break; }
        default: {
#ifndef NDEBUG
            throw std::runtime_error("Unknown Cache Hint!");
#endif //NDEBUG
            break;
        }
    }

#endif // __SSE__ && (__GNUC__ || __clang__ )

}

#endif //CHDR_INTRINSICS_HPP