/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

// ReSharper disable CppInconsistentNaming

#ifndef CHDR_STACKALLOCATOR_HPP
#define CHDR_STACKALLOCATOR_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <type_traits>

// ReSharper disable once CppUnusedIncludeDirective
#include <stdexcept> // NOLINT(*-include-cleaner)

namespace chdr {

    template <typename T, uintptr_t StackSize>
    class stack_allocator {

    private:

        alignas(T) std::byte dataa[StackSize * sizeof(T)]; // NOLINT(*-avoid-c-arrays)
        uintptr_t data_ptr;

    public:

        using value_type [[maybe_unused]] = T;

        // ReSharper disable once CppPossiblyUninitializedMember
        constexpr stack_allocator() : data_ptr(0U) {}

        [[maybe_unused, nodiscard]] constexpr T* allocate(const uintptr_t& _n) {

            T* result;

            if (data_ptr + _n <= StackSize) {
                result = reinterpret_cast<T*>(dataa.data() + (data_ptr * sizeof(T)));
                data_ptr += _n;
            }
            else {
                result = std::allocator<T>().allocate(_n);
            }

            return result;
        }

        constexpr void deallocate(T* _p, const uintptr_t& _n) {

            if (_p >= reinterpret_cast<T*>(dataa.data()) && _p < reinterpret_cast<T*>(dataa.data() + dataa.size())) {

#ifndef NDEBUG
                if (data_ptr < _n) {
                    throw std::runtime_error("Deallocate called with too large n");
                }
#endif //NDEBUG

                data_ptr -= _n;
            }
            else {
                std::allocator<T>().deallocate(_p, _n);
            }
        }

        template <typename U>
        struct [[maybe_unused]] rebind {
            using other [[maybe_unused]] = stack_allocator<U, StackSize>;
        };

        using propagate_on_container_move_assignment = std::true_type;
        using is_always_equal                        = std::true_type;
    };

} //chdr

#endif //CHDR_STACKALLOCATOR_HPP