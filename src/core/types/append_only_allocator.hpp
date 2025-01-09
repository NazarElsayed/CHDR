/*
* Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_APPEND_ONLY_ALLOCATOR_HPP
#define CHDR_APPEND_ONLY_ALLOCATOR_HPP

#include <cmath>
#include <cstddef>
#include <forward_list>
#include <memory>
#include <type_traits>

namespace chdr {

    template <typename T>
    class append_only_allocator {

    private:

        using block_t = std::unique_ptr<T[]>; // NOLINT(*-avoid-c-arrays)

        static constexpr size_t max_block_width { 65536U / 4U };

        size_t block_width;
        size_t index;

        std::forward_list<block_t> c;

        constexpr void expand() {

            c.emplace_front(std::make_unique<T[]>(block_width)); // NOLINT(*-avoid-c-arrays)
            index = 0U;

            block_width = std::min(block_width * 2U, max_block_width);
        }

    public:

        using value_type [[maybe_unused]] = T;

        constexpr append_only_allocator(const size_t& _capacity = 64U) :
            block_width(std::min(_capacity, max_block_width)),
            index(block_width / 2U)
        {
            if (_capacity == 0U) {
                throw std::invalid_argument("Capacity cannot be zero.");
            }
        }

        template <typename U>
        constexpr append_only_allocator([[maybe_unused]] const append_only_allocator<U>& _other) noexcept :
            block_width(std::min(static_cast<size_t>(64U), max_block_width)),
            index(block_width / 2U) {}

        void construct(T* _p, T&& _val) {
            if (_p != nullptr) {
                new(_p) T(std::move(_val));
            }
        }

        template <typename... Args>
        void construct(T* _p, Args&&... _args) {
            if (_p != nullptr) {
                new(_p) T(std::forward<Args>(_args)...);
            }
        }

        [[nodiscard]] constexpr T* allocate([[maybe_unused]] const uintptr_t& _n) {

            if (index == block_width / 2U) {
                expand();
            }

            return &c.front().get()[index++];
        }

        static constexpr void deallocate([[maybe_unused]] T* _p, [[maybe_unused]] const uintptr_t& _n) noexcept {
            static_assert(true, "Allocator is append-only.");
        }

        template <typename U>
        struct [[maybe_unused]] rebind {
            using other [[maybe_unused]] = append_only_allocator<U>;
        };

        template <typename Alloc>
        struct [[maybe_unused]] allocator_rebind {
            template <typename U>
            using other [[maybe_unused]] = typename Alloc::template rebind<U>::other;
        };

        using propagate_on_container_move_assignment = std::true_type;
        using is_always_equal                        = std::true_type;
    };

} //chdr

#endif // CHDR_APPEND_ONLY_ALLOCATOR_HPP