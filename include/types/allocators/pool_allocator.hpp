/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

// ReSharper disable CppInconsistentNaming

#ifndef CHDR_DYNAMIC_POOL_ALLOCATOR_HPP
#define CHDR_DYNAMIC_POOL_ALLOCATOR_HPP

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <type_traits>
#include <vector>

#include "base/arena.hpp"

// ReSharper disable once CppUnusedIncludeDirective
#include "../../utils/intrinsics.hpp" // NOLINT(*-include-cleaner)

namespace chdr {

    template <typename T>
    class pool_allocator {

    private:

        using block_t = arena<T>;

        static constexpr size_t     max_block_width { 65536U / sizeof(T*) };
        static constexpr size_t default_block_width { utils::min(static_cast<size_t>(32U), max_block_width) };

        size_t initial_block_width;
        size_t block_width;

        std::vector<block_t> pool;
        std::vector<T*>      free;

        constexpr const auto& expand(const block_t& _new_block, const size_t& _skip_first = 0U) {

            assert(_new_block.size() > _skip_first && "Underflow detected: _new_block.size() must be greater than _skip_first.");

            const auto count = _new_block.size() - _skip_first;

            assert(count != 0U && "Count must not be 0.");

            free.reserve(free.size() + count);

            IVDEP
            for (size_t i = 0U; i != count; ++i) {
                free.emplace_back(_new_block.get() + _skip_first + i);
            }

            return _new_block;
        }

    public:

        using value_type [[maybe_unused]] = T;

        explicit constexpr pool_allocator() noexcept :
            initial_block_width(default_block_width),
            block_width        (initial_block_width) {}

        explicit constexpr pool_allocator(const size_t& _capacity) noexcept :
            initial_block_width(utils::min(_capacity, max_block_width)),
            block_width        (initial_block_width)
        {
            assert(_capacity >= 2U && "Capacity must be at least 2.");
        }

        template <typename U>
        constexpr pool_allocator([[maybe_unused]] const pool_allocator<U>& _other) noexcept :
            initial_block_width(default_block_width),
            block_width        (initial_block_width) {}

        constexpr pool_allocator(const pool_allocator& _other) noexcept = default;
        constexpr pool_allocator(      pool_allocator& _other) noexcept = default;

        constexpr pool_allocator& operator=(const pool_allocator&  _other) noexcept = default;
        constexpr pool_allocator& operator=(      pool_allocator&& _other) noexcept = default;
        
        void construct(T* _p, const T& _val) {
            static_assert(std::is_copy_constructible_v<T>, "T must be copy constructible.");
            assert(_p != nullptr && "Attempting to construct at a null pointer.");
            new(_p) T(_val);
        }

        void construct(T* _p, T&& _val) {
            static_assert(std::is_move_constructible_v<T>, "T must be move constructible.");
            assert(_p != nullptr && "Attempting to construct at a null pointer.");
            new(_p) T(std::move(_val));
        }

        template <typename... Args>
        void construct(T* _p, Args&&... _args) {
            static_assert(std::is_constructible_v<T, Args...>, "T cannot be constructed with the provided arguments.");
            assert(_p != nullptr && "Attempting to construct at a null pointer.");
            new(_p) T(std::forward<Args>(_args)...);
        }
        
        [[nodiscard]] T* allocate([[maybe_unused]] const uintptr_t& _n) {

            assert(_n != 0U && "Tried to allocate 0 objects.");

            if (_n == 1U) {

                T* result;

                if (free.empty()) {
                    result = expand(pool.emplace_back(block_width), 1U).get();
                    block_width = utils::min(block_width * 2U, max_block_width);
                }
                else {
                    result = free.back();
                    free.pop_back();
                }

                return result;
            }

            return expand(pool.emplace_back(_n), 1U).get();
        }

        void deallocate(T* _p, [[maybe_unused]] const uintptr_t& _n) {

            assert(_p != nullptr && "Attempt to deallocate a null pointer.");

            assert(_n != 0U && "Tried to allocate 0 objects.");
            assert(_n == 1U && "Does not support batch deallocation.");

            if (_n == 1U) {
                free.emplace_back(_p);
            }
            else {
                free.reserve(free.size() + _n);

                IVDEP
                for (size_t i = 0U; i != _n; ++i) {
                    free.emplace_back(_p + i);
                }
            }
        }

        void release() noexcept {

            block_width = initial_block_width;

            free.clear();

            try {
                for (const auto& block : pool) {

                    free.reserve(free.size() + block.m_size);

                    IVDEP
                    for (size_t i = 0U; i != block.m_size; ++i) {
                        free.emplace_back(block.get() + i);
                    }
                }
            }
            catch (...) {
                try {
                    free = std::move(decltype(free){});
                    pool = std::move(decltype(pool){});
                }
                catch (...) {
                    free.clear(); free.shrink_to_fit();
                    pool.clear(); pool.shrink_to_fit();
                }
            }
        }

        void reset() noexcept {

            block_width = initial_block_width;

            try {
                free = std::move(decltype(free){});
                pool = std::move(decltype(pool){});
            }
            catch (...) {
                free.clear(); free.shrink_to_fit();
                pool.clear(); pool.shrink_to_fit();
            }
        }

        constexpr bool operator == (const pool_allocator& _other) const noexcept { return    this == &_other; }
        constexpr bool operator != (const pool_allocator& _other) const noexcept { return !(*this == _other); } // NOLINT(*-simplify)

        template <typename U>
        struct rebind {
            using other = pool_allocator<U>;
        };

        template <typename U, typename Alloc>
        struct [[maybe_unused]] allocator_rebind {
            using other = typename std::allocator_traits<Alloc>::template rebind_alloc<U>;
        };

        using propagate_on_container_copy_assignment = std::false_type;
        using propagate_on_container_move_assignment = std::true_type;
        using propagate_on_container_swap            = std::true_type;
        using is_always_equal                        = std::is_empty<T>;
    };

} //chdr

#endif //CHDR_DYNAMIC_POOL_ALLOCATOR_HPP