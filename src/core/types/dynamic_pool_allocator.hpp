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

        static constexpr size_t max_block_width { 65536U / 4U };

        size_t initial_block_width;
        size_t block_width;
        T*     current_block_start;
        T*     current_block_end;

        std::forward_list<block_t> c;
        std::vector<T*> free;

        constexpr void expand() {
            current_block_start = c.emplace_front(std::make_unique<T[]>(block_width)).get(); // NOLINT(*-avoid-c-arrays);
            current_block_end   = current_block_start + block_width;

            block_width = std::min(block_width * 2U, max_block_width);
        }

    public:
        using value_type [[maybe_unused]] = T;

        constexpr dynamic_pool_allocator(const size_t& _capacity = 32U) noexcept :
            initial_block_width (std::min(_capacity, max_block_width)),
            block_width         (initial_block_width                 ),
            current_block_start (nullptr                             ),
            current_block_end   (nullptr                             )
        {
            assert(_capacity != 0U && "Capacity cannot be zero.");
        }

        void construct(T* _p, T&& _val) noexcept {
            assert(_p != nullptr && "Nullptr deferencing.");
            new(_p) T(std::move(_val));
        }

        template <typename... Args>
        void construct(T* _p, Args&&... _args) {

            static_assert(std::is_constructible_v<T, Args...>, "Object type cannot be constructed with the provided arguments");

            assert(_p != nullptr && "Nullptr deferencing.");
            new(_p) T(std::forward<Args>(_args)...);
        }

        [[nodiscard]] T* allocate([[maybe_unused]] const uintptr_t& _n) {

            T* result;

            if (!free.empty()) {
                result = free.back();
                free.pop_back();
            }
            else {
                if (current_block_start == current_block_end) {
                    expand();
                }
                result = current_block_start++;
            }

            return result;
        }

        void deallocate(T* _p, [[maybe_unused]] const uintptr_t& _n) noexcept {
            assert(_p != nullptr && "Nullptr deferencing.");
            free.emplace_back(_p);
        }

        void reset() noexcept {
            block_width         = initial_block_width;
            current_block_start = nullptr;
            current_block_end   = nullptr;

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