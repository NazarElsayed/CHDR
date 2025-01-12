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

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <list>
#include <type_traits>
#include <vector>

#include "../utils/intrinsics.hpp"

namespace chdr {

    template <typename T>
    class dynamic_pool_allocator {
    
    private:

        using block_t = std::vector<T>;

        static constexpr size_t max_block_width { 65536U / sizeof(T*) };

        size_t initial_block_width;
        size_t block_width;

        std::list<block_t> c;
        std::vector<T*> free;

        constexpr void expand(T* const _new_block, const size_t& _skip_first) { // NOLINT(*-unused-parameters)

            free.resize(free.size() + (block_width - _skip_first), {});

            IVDEP
            for (size_t i = 0U; i != block_width - _skip_first; ++i) {
                free[i] = &_new_block[block_width - _skip_first - i];
            }

            block_width = std::min(block_width * 2U, max_block_width);
        }
        
    public:

        using value_type [[maybe_unused]] = T;

        explicit constexpr dynamic_pool_allocator() noexcept :
            initial_block_width(std::min(static_cast<size_t>(16U), max_block_width)),
            block_width(initial_block_width)
        {}

        ~dynamic_pool_allocator() noexcept {
            free = {};
               c = {};
        }

        explicit constexpr dynamic_pool_allocator(const size_t& _capacity) noexcept :
            initial_block_width(std::min(_capacity, max_block_width)),
            block_width(initial_block_width)
        {
            assert(_capacity != 0U && "Capacity cannot be zero.");
        }

        template <typename U>
        constexpr dynamic_pool_allocator([[maybe_unused]] const dynamic_pool_allocator<U>& _other) noexcept :
            initial_block_width(std::min(static_cast<size_t>(16U), max_block_width)),
            block_width(initial_block_width) {}

        template <typename Alloc>
        constexpr dynamic_pool_allocator(const dynamic_pool_allocator& _other, const Alloc& _custom_allocator) noexcept :
            initial_block_width(_other.initial_block_width),
            block_width(_other.block_width),
            c(_other.c.begin(), _other.c.end(), _custom_allocator),
            free(_other.free) {}

        constexpr dynamic_pool_allocator(dynamic_pool_allocator& _other) noexcept :
            initial_block_width(_other.initial_block_width),
            block_width(_other.block_width),
            c(_other.c),
            free(_other.free) {}

        constexpr dynamic_pool_allocator(const dynamic_pool_allocator& _other) noexcept :
            initial_block_width(_other.initial_block_width),
            block_width(_other.block_width),
            c(_other.c),
            free(_other.free) {}

        constexpr dynamic_pool_allocator& operator=(dynamic_pool_allocator& other) noexcept {
            if (this != &other) {
                initial_block_width = other.initial_block_width;
                block_width         = other.block_width;
                c                   = other.c;
                free                = other.free;
            }
            return *this;
        }

        constexpr dynamic_pool_allocator& operator=(dynamic_pool_allocator&& other) noexcept {
            if (this != &other) {
                initial_block_width = other.initial_block_width;
                block_width         = other.block_width;
                c                   = std::move(other.c);
                free                = std::move(other.free);

                // Reset the other allocator to its initial state
                other.reset();
            }
            return *this;
        }
        
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
            assert(_n == 1U && "Does not support batch allocation.");

            T* result;

            if (free.empty()) {
                expand(result = &c.emplace_front(block_t(block_width))[0U], 1U);
            }
            else {
                result = free.back();
                free.pop_back();
            }

            return result;
        }

        void deallocate(T* _p, [[maybe_unused]] const uintptr_t& _n) noexcept {

            assert(_p != nullptr && "Attempt to deallocate a null pointer.");

            assert(_n != 0U && "Tried to allocate 0 objects.");
            assert(_n == 1U && "Does not support batch deallocation.");

            free.emplace_back(_p);
        }

        void release() noexcept {

            block_width = initial_block_width;

            free.clear();

            try {
                for (auto& block : c) {
                    for (auto& item : block) {
                        free.emplace_back(&item);
                    }
                }
            }
            catch ([[maybe_unused]] const std::exception& e) {
                free = {};
                   c = {};
            }
        }

        void reset() noexcept {

            block_width = initial_block_width;

            free = {};
               c = {};
        }

        constexpr bool operator==(const dynamic_pool_allocator& other) const noexcept { return    this == &other; }
        constexpr bool operator!=(const dynamic_pool_allocator& other) const noexcept { return !(*this == other); }

        template <typename U>
        struct rebind {
            using other = dynamic_pool_allocator<U>;
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