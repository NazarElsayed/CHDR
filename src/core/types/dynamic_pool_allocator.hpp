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

        static constexpr size_t max_block_width { 65536U / sizeof(T*) };

        size_t initial_block_width;
        size_t block_width;

        std::forward_list<block_t> c;
        std::vector<T*> free;

        constexpr void expand() {

            const auto& new_block = c.emplace_front(std::make_unique<T[]>(block_width)); // NOLINT(*-avoid-c-arrays)

            constexpr size_t skip_first { 1U };

            free.resize(free.size() + (block_width - skip_first), {});

            IVDEP
            VECTOR_ALWAYS
            for (size_t i = 0U; i != block_width - skip_first; ++i) {
                free[i] = &new_block[block_width - skip_first - i];
            }

            block_width = std::min(block_width * 2U, max_block_width);
        }

    public:

        using value_type [[maybe_unused]] = T;

        constexpr dynamic_pool_allocator(const size_t& _capacity = 16U) noexcept :
            initial_block_width(std::min(_capacity, max_block_width)),
            block_width(initial_block_width)
        {
            assert(_capacity != 0U && "dynamic_pool_allocator: Capacity cannot be zero.");
        }

        template <typename... Args>
        void construct(T* _p, Args&&... _args) {

            static_assert(std::is_constructible_v<T, Args...>, "Object type cannot be constructed with the provided arguments");

            assert(_p != nullptr                && "dynamic_pool_allocator: Attempt to construct at a null pointer.");
            assert(!_p || c.front().get() <= _p || "dynamic_pool_allocator: Pointer does not belong to the pool.");

            new(_p) T(std::forward<Args>(_args)...);
        }

        [[nodiscard]] T* allocate([[maybe_unused]] const uintptr_t& _n) {

            assert(_n != 0U && "dynamic_pool_allocator: Tried to allocate 0 objects.");
            assert(_n == 1U && "dynamic_pool_allocator: Does not support batch allocation.");
            
            T* result;

            if (free.empty()) {
                expand();
                result = c.front().get();
            }
            else {
                result = free.back();
                free.pop_back();
            }

            return result;
        }

        void deallocate(T* _p, [[maybe_unused]] const uintptr_t& _n) noexcept {

            assert(!_p || c.front().get() <= _p || "dynamic_pool_allocator: Pointer does not belong to the pool.");

            assert(_n != 0U && "dynamic_pool_allocator: Tried to allocate 0 objects.");
            assert(_n == 1U && "dynamic_pool_allocator: Does not support batch deallocation.");

            free.emplace_back(_p);
        }

        void reset() noexcept {

            block_width = initial_block_width;

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