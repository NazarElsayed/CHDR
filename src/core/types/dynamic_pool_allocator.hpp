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
#include <memory>
#include <type_traits>
#include <forward_list>
#include <vector>

namespace chdr {

    template <typename T>
    class dynamic_pool_allocator {

    private:

        using block_t = std::unique_ptr<T[]>; // NOLINT(*-avoid-c-arrays)

        size_t initial_capacity;
        size_t capacity;

        static constexpr size_t max_capacity = 4096U;

        std::forward_list<block_t> c;
        std::vector<T*> free;

        void expand() {
            const auto& new_block = c.emplace_front(std::make_unique<T[]>(capacity)); // NOLINT(*-avoid-c-arrays)

            free.reserve(free.size() + capacity);
            for (size_t i = 0U; i < capacity; ++i) {
                free.emplace_back(&new_block[i]);
            }

            capacity = std::min(capacity * 2U, max_capacity);
        }

    public:

        using value_type [[maybe_unused]] = T;

        constexpr dynamic_pool_allocator(const size_t& _capacity = 16U) :
            initial_capacity(_capacity),
            capacity(_capacity) {}

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

            static_assert(std::is_constructible_v<T, Args...>, "Object type cannot be constructed with the provided arguments");

            if (_p != nullptr) {
                new(_p) T(std::forward<Args>(_args)...);
            }
        }

        [[nodiscard]] T* allocate([[maybe_unused]] const uintptr_t& _n) {

            if (free.empty()) {
                expand();
            }

            T* result(free.back());
            free.pop_back();

            return result;
        }

        void deallocate(T* _p, [[maybe_unused]] const uintptr_t& _n) {
            
            if (_p != nullptr) {
                free.emplace_back(_p);
            }
        }

        void reset() {

            capacity = initial_capacity;

               c.clear();
            free.clear();
            free.shrink_to_fit();
        }
        
        template <typename U>
        struct [[maybe_unused]] rebind {
            using other [[maybe_unused]] = dynamic_pool_allocator<U>;
        };

        using propagate_on_container_move_assignment = std::true_type;
        using is_always_equal                        = std::true_type;
    };

} //chdr

#endif //CHDR_DYNAMIC_POOL_ALLOCATOR_HPP