/*
* Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_APPEND_ONLY_ALLOCATOR_HPP
#define CHDR_APPEND_ONLY_ALLOCATOR_HPP

#include <cstddef>
#include <cstdint>
#include <memory>
#include <type_traits>

namespace chdr {

    template <typename T>
    class append_only_allocator final {

    private:

        using block_t = std::unique_ptr<T[]>; // NOLINT(*-avoid-c-arrays)

        static constexpr size_t     max_block_width { 65536U / sizeof(T*) };
        static constexpr size_t initial_block_width { utils::min(static_cast<size_t>(64U), max_block_width) };

        static_assert(initial_block_width >= 2, "initial_block_width must be at least 2.");
        
        size_t block_width;
        size_t index;

        std::vector<block_t> c;

        constexpr void expand() {

            c.emplace_back(std::make_unique<T[]>(block_width)); // NOLINT(*-avoid-c-arrays)
            index = 0U;

            block_width = utils::min(block_width * 2U, max_block_width);
        }

    public:

        using value_type [[maybe_unused]] = T;

        constexpr append_only_allocator() noexcept :
            block_width(initial_block_width),
            index(block_width / 2U) {}

        ~append_only_allocator() = default;

        template <typename U>
        constexpr append_only_allocator([[maybe_unused]] const append_only_allocator<U>& _other) noexcept :
            block_width(initial_block_width),
            index(block_width / 2U) {}

        template <typename Alloc>
        constexpr append_only_allocator(const append_only_allocator& _other, [[maybe_unused]] const Alloc& _custom_allocator) noexcept :
            block_width(_other.block_width),
            index(_other.index) {}

        constexpr append_only_allocator(append_only_allocator& _other) noexcept :
            block_width(_other.block_width),
            index(_other.index) {}

        constexpr append_only_allocator(const append_only_allocator& _other) noexcept :
            block_width(_other.block_width),
            index(_other.index) {}

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

        [[nodiscard]] constexpr T* allocate([[maybe_unused]] const uintptr_t& _n) {

            assert(_n != 0U && "Tried to allocate 0 objects.");
            assert(_n == 1U && "Does not support batch allocation.");

            if (index == block_width / 2U) {
                expand();
            }

            return &c.back().get()[index++];
        }

        static constexpr void deallocate([[maybe_unused]] T* _p, [[maybe_unused]] const uintptr_t& _n) noexcept {
            static_assert(true, "Allocator is append-only.");
        }

        void release() noexcept {

            block_width = initial_block_width;
            index       = block_width / 2U;

            c.clear();
        }

        void reset() noexcept {

            block_width = initial_block_width;
            index       = block_width / 2U;

            c.clear();
            c.shrink_to_fit();
        }

        constexpr bool operator==(const append_only_allocator& _other) const noexcept { return    this == &_other; }
        constexpr bool operator!=(const append_only_allocator& _other) const noexcept { return !(*this == _other); } // NOLINT(*-simplify)

        template <typename U>
        struct rebind {
            using other = append_only_allocator<U>;
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

#endif // CHDR_APPEND_ONLY_ALLOCATOR_HPP