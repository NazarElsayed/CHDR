/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_BUMP_ALLOCATOR_HPP
#define CHDR_BUMP_ALLOCATOR_HPP

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <type_traits>

#include "base/memory_block.hpp"

namespace chdr {

    template <typename T>
    class bump_allocator final {

    private:

        using block_t = memory_block<T>;

        static constexpr size_t     max_block_width { 65536U / sizeof(T*) };
        static constexpr size_t initial_block_width { utils::min(static_cast<size_t>(32U), max_block_width) };

        static_assert(initial_block_width >= 2U, "initial_block_width must be at least 1.");

        size_t block_width; // Width of next block to create.
        size_t block_index; // Index of current write block.
        size_t block_write; // Head (write position) within the current write block.

        std::vector<block_t> c; // Collection of allocated blocks.

        constexpr void expand() {
            c.emplace_back(block_width);
            block_width = utils::min(block_width * 2U, max_block_width);
            block_index = c.size() - 1U;
            block_write = 0U;
        }

    public:

        using value_type [[maybe_unused]] = T;

        constexpr bump_allocator() noexcept :
            block_width(initial_block_width),
            block_index(0U),
            block_write(0U) {}

        template <typename U>
        constexpr bump_allocator([[maybe_unused]] const bump_allocator<U>& _other) noexcept :
            block_width(initial_block_width),
            block_index(0U),
            block_write(0U) {}

        constexpr bump_allocator(const bump_allocator& ) noexcept = default;
        constexpr bump_allocator(      bump_allocator&&) noexcept = default;

        constexpr bump_allocator& operator=(      bump_allocator&&) noexcept = default;
        constexpr bump_allocator& operator=(const bump_allocator& ) noexcept = default;

        void construct(T* _p, const T& _val) {
            static_assert(std::is_copy_constructible_v<T>, "T must be copy constructible.");
            assert(_p != nullptr && "Attempting to construct at a null pointer.");
            new (_p) T(_val);
        }

        void construct(T* _p, T&& _val) {
            static_assert(std::is_move_constructible_v<T>, "T must be move constructible.");
            assert(_p != nullptr && "Attempting to construct at a null pointer.");
            new (_p) T(std::move(_val));
        }

        template <typename... Args>
        void construct(T* _p, Args&&... _args) {
            static_assert(std::is_constructible_v<T, Args...>, "T cannot be constructed with the provided arguments.");
            assert(_p != nullptr && "Attempting to construct at a null pointer.");
            new (_p) T(std::forward<Args>(_args)...);
        }

        [[nodiscard]] constexpr T* allocate([[maybe_unused]] const uintptr_t& _n) {

            assert(_n != 0U && "Tried to allocate 0 objects.");
            assert(_n == 1U && "Does not support batch allocation.");

            if (c.empty() || block_write >= c[block_index].m_size) {
                if (c.empty() || block_index + 1U >= c.size()) {
                    expand();
                }
                else {
                    block_index++;
                    block_write = 0U;
                }
            }

            return &c[block_index].get()[block_write++];
        }

        constexpr void deallocate([[maybe_unused]] T* _p, [[maybe_unused]] const uintptr_t& _n) {

            assert(_p != nullptr && "Attempt to deallocate a null pointer.");

            /*
             * If _p is within the bounds of the current block,
             * move the head back to allow overwriting.
             */

            T* block_ptr = c[block_index].get();

            if (_p >= block_ptr && _p < block_ptr + c[block_index].m_size) {
                
                if (_p == block_ptr + block_write - 1U) {
                    --block_write;
                }
                else {
                    throw std::invalid_argument("Deallocation must originate from the head of the current write block.");
                }
            }
            else {
                throw std::out_of_range("Deallocation must take place within the most recently allocated memory block.");
            }
        }

        void release() noexcept {
            block_index = 0U;
            block_write = 0U;
        }

        void reset() noexcept {
            block_width = initial_block_width;
            block_write = 0U;
            block_index = 0U;
            c = std::move(decltype(c){});
        }

        constexpr bool operator == (const bump_allocator& _other) const noexcept { return    this == &_other; }
        constexpr bool operator != (const bump_allocator& _other) const noexcept { return !(*this == _other); }

        template <typename U>
        struct rebind {
            using other = bump_allocator<U>;
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

#endif // CHDR_BUMP_ALLOCATOR_HPP