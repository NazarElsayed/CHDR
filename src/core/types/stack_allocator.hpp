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

#include <cstddef>
#include <memory>

template <typename T, size_t StackSize>
class stack_allocator {

private:

    alignas(T) char m_stack[StackSize * sizeof(T)];

    size_t m_stack_ptr;

public:

    using value_type [[maybe_unused]] = T;

    // ReSharper disable once CppPossiblyUninitializedMember
    stack_allocator() : m_stack_ptr(0U) {}

    [[maybe_unused]] T* allocate(const size_t& _n) {
        T* result;

        if (m_stack_ptr + _n <= StackSize) {
            result = reinterpret_cast<T*>(m_stack + (m_stack_ptr * sizeof(T)));
            m_stack_ptr += _n;
        }
        else {
            result = std::allocator<T>().allocate(_n);
        }

        return result;
    }

    [[maybe_unused]] void deallocate(T* _p, const size_t& _n) {
        if (_p >= reinterpret_cast<T*>(m_stack) && _p < reinterpret_cast<T*>(m_stack + sizeof(m_stack))) {
            m_stack_ptr -= _n;
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

#endif //CHDR_STACKALLOCATOR_HPP