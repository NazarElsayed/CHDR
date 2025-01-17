/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_RAW_BLOCK_HPP
#define CHDR_RAW_BLOCK_HPP

#include <cassert>
#include <cstddef>
#include <cstdlib>

#include <new>

// Define fallback CACHE_LINE_SIZE if std::hardware_destructive_interference_size is unavailable
#if __has_include(<new>) && defined(__cpp_lib_hardware_interference_size)
    #define CACHE_LINE_SIZE std::hardware_destructive_interference_size
#else
    // Fallback value, assuming 64 bytes cache line size for most modern processors.
    #define CACHE_LINE_SIZE 64
#endif

#if defined(_WIN32)
    #include <malloc.h>
#endif

#include "../../../utils/intrinsics.hpp"

namespace chdr {

    template <typename T>
    struct alignas(CACHE_LINE_SIZE) arena final {

    private:

        static constexpr size_t s_alignment { utils::max(alignof(T), CACHE_LINE_SIZE) };

        size_t      m_size;
        T* RESTRICT m_data;

        static void* aligned_allocate(const size_t& _alignment, const size_t& _size) {

#if defined(_WIN32)
            void* ptr = _aligned_malloc(_size, _alignment);
            if (ptr == nullptr) { throw std::bad_alloc(); }
#elif defined(__linux__) || defined(__APPLE__)
            void* ptr;
            if (posix_memalign(&ptr, _alignment, _size) != 0) { throw std::bad_alloc(); }
#else
            void* ptr = std::aligned_alloc(_alignment, _size);
            if (ptr == nullptr) { throw std::bad_alloc(); }
#endif

            return ptr;
        }

        static void aligned_deallocate(void* _ptr) noexcept {

#if defined(_WIN32)
            _aligned_free(_ptr);
#else
            std::free(_ptr);
#endif
        }

    public:

        [[maybe_unused, nodiscard]] constexpr arena() noexcept :
            m_size(0U),
            m_data(nullptr) {}

        [[maybe_unused, nodiscard]] arena(const size_t& _size) :
            m_size(_size),
            m_data(static_cast<T*>(aligned_allocate(s_alignment, (_size * sizeof(T) + s_alignment - 1U) & ~(s_alignment - 1U)))) {}

        [[maybe_unused]] ~arena() noexcept {
            aligned_deallocate(m_data);
        }

        arena           (const arena&) = delete;
        arena& operator=(const arena&) = delete;

        [[maybe_unused, nodiscard]] constexpr arena(arena&& _other) noexcept :
            m_size(std::exchange(_other.m_size, 0U)),
            m_data(std::exchange(_other.m_data, nullptr)) {}

        [[maybe_unused, nodiscard]] constexpr arena& operator=(arena&& _other) noexcept {

            if (this != &_other) {
                aligned_deallocate(m_data);
                m_data = std::exchange(_other.m_data, nullptr);
                m_size = std::exchange(_other.m_size, 0U);
            }
            return *this;
        }

        [[nodiscard]] constexpr const auto& size() const noexcept { return m_size; }
        [[nodiscard]] constexpr       auto*  get() const noexcept { return m_data; }

        [[nodiscard]] constexpr bool is_initialized() const noexcept {
            return m_data != nullptr;
        }

        [[maybe_unused]] void reset() noexcept {
            aligned_deallocate(m_data);
            m_size = 0U;
        }

        [[maybe_unused, nodiscard]] constexpr T& operator[](const size_t& _index) noexcept {
            assert(_index < m_size && "Index out of bounds");
            return m_data[_index];
        }

        [[maybe_unused, nodiscard]] constexpr const T& operator[](const size_t& _index) const noexcept {
            assert(_index < m_size && "Index out of bounds");
            return m_data[_index];
        }

        constexpr bool operator == (const arena& _other) const noexcept { return    this == &_other; }
        constexpr bool operator != (const arena& _other) const noexcept { return !(*this == _other); } // NOLINT(*-simplify)

    };

} //chdr

#undef CACHE_LINE_SIZE

#endif //CHDR_RAW_BLOCK_HPP