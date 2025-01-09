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

namespace chdr {

    template <typename T>
    class dynamic_pool_allocator final {
    
    private:

        using block_t = std::vector<T>;

        static constexpr size_t max_block_width { 65536U / sizeof(T*) };

        size_t initial_block_width;
        size_t block_width;

        std::list<block_t> c;
        std::vector<T*> free;

        constexpr void expand(T* const _new_block, const size_t& _skip_first) {

            free.resize(free.size() + (block_width - _skip_first), {});

            IVDEP
            for (size_t i = 0U; i != block_width - _skip_first; ++i) {
                free[i] = &_new_block[block_width - _skip_first - i];
            }

            block_width = std::min(block_width * 2U, max_block_width);
        }

    public:

        using value_type [[maybe_unused]] = T;

        constexpr dynamic_pool_allocator(const size_t& _capacity = 16U) noexcept :
            initial_block_width(std::min(_capacity, max_block_width)),
            block_width(initial_block_width)
        {
            assert(_capacity != 0U && "Capacity cannot be zero.");
        }

        template <typename U>
        constexpr dynamic_pool_allocator([[maybe_unused]] const dynamic_pool_allocator<U>& _other) noexcept :
            initial_block_width(std::min(static_cast<size_t>(16U), max_block_width)),
            block_width(initial_block_width) {}

        template <typename... Args>
        void construct(T* _p, Args&&... _args) noexcept {

            static_assert(std::is_constructible_v<T, Args...>, "Object type cannot be constructed with the provided arguments");

            assert(_p != nullptr && "Attempt to construct at a null pointer.");

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
            catch ([[maybe_unused]] const std::exception& _e) {
                free = {};
                   c = {};
            }
        }

        void reset() noexcept {

            block_width = initial_block_width;

            free = {};
               c = {};
        }

        template <typename U>
        struct [[maybe_unused]] rebind {
            using other [[maybe_unused]] = dynamic_pool_allocator<U>;
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

#endif //CHDR_DYNAMIC_POOL_ALLOCATOR_HPP