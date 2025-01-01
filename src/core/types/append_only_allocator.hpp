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
#include <forward_list>
#include <memory>
#include <type_traits>

namespace chdr {

    template <typename T, size_t BlockWidth = 4096U>
    class append_only_allocator {

        static_assert(BlockWidth >= 1U, "BlockWidth must be at least 1 or greater.");
        
    private:

        using block_t = std::unique_ptr<T[]>; // NOLINT(*-avoid-c-arrays)

        std::forward_list<block_t> c;
        size_t index;

        void expand() {
            c.emplace_front(std::make_unique<T[]>(BlockWidth)); // NOLINT(*-avoid-c-arrays)
            index = 0U;
        }

    public:

        using value_type [[maybe_unused]] = T;

        // ReSharper disable once CppPossiblyUninitializedMember
        constexpr append_only_allocator() : index(BlockWidth) {} // NOLINT(*-pro-type-member-init, *-use-equals-default)

        template <typename... Args>
        [[nodiscard]] T* allocate_and_construct(Args&&... _args) {

            static_assert(std::is_constructible_v<T, Args...>, "Object type cannot be constructed with the provided arguments");

            T* memory { allocate(1U) };
            construct(memory, std::forward<Args>(_args)...);
            return memory;
        }

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

        [[nodiscard]] T* allocate([[maybe_unused]] const uintptr_t& _n) {

            if (index == BlockWidth) {
                expand();
            }

            return &c.front().get()[index++];
        }

        void deallocate(T* _p, [[maybe_unused]] const uintptr_t& _n) {
            static_assert(true, "Allocator is append-only.");
        }

        template <typename U>
        struct [[maybe_unused]] rebind {
            using other [[maybe_unused]] = append_only_allocator<U>;
        };

        using propagate_on_container_move_assignment = std::true_type;
        using is_always_equal                        = std::true_type;
    };

} //chdr

#endif // CHDR_APPEND_ONLY_ALLOCATOR_HPP