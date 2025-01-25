/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

// ReSharper disable CppInconsistentNaming

#ifndef CHDR_POLYTONIC_POOL_HPP
#define CHDR_POLYTONIC_POOL_HPP

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <memory_resource>
#include <set>
#include <utility>
#include <vector>

#include "../../utils/intrinsics.hpp" // NOLINT(*-include-cleaner)
#include "../../utils/utils.hpp"

namespace chdr {

    class polytonic_pool : public std::pmr::memory_resource {

    private:

        static constexpr size_t default_block_width {  4096U };
        static constexpr size_t     max_block_width { 65536U };
        static constexpr size_t  s_stack_block_size {  4096U };

        alignas(std::max_align_t) uint8_t m_stack_block[s_stack_block_size];

        size_t initial_block_width;
        size_t m_stack_write;
        size_t block_width;

        struct block final {

            size_t   size;
            size_t   alignment;
            uint8_t* data;

            [[nodiscard]] HOT constexpr block(size_t _size, size_t _alignment, uint8_t* _data) noexcept :
                size     (_size),
                alignment(_alignment),
                data     (_data) {}

            ~block() = default;

            [[nodiscard]] HOT constexpr block           (const block&) = default;
                          HOT constexpr block& operator=(const block&) = default;

            [[nodiscard]] HOT constexpr block(block&& other) noexcept = default;

#if __cplusplus > 202302L
            constexpr
#endif
            block& operator=(block&& other) noexcept = default;

            [[nodiscard]] HOT constexpr bool operator<(const block& other) const noexcept {
                return size < other.size;
            }
        };

        std::vector<block> blocks;
        std::set   <block> free;

        HOT void expand(size_t _bytes, size_t _alignment) {

            const auto allocate_size = utils::max(block_width, _bytes);

            free.insert(
                blocks.emplace_back(
                    allocate_size,
                    _alignment,
                    static_cast<uint8_t*>(::operator new(allocate_size, static_cast<std::align_val_t>(_alignment)))
                )
            );

            block_width = utils::min(block_width * 2U, max_block_width);
        }

        HOT uint8_t* allocate_from_free(const size_t _bytes, const size_t _alignment) {

            uint8_t* result;

            // Find the smallest free block that fits:
            auto it = free.lower_bound(block { _bytes, _alignment, nullptr });
            if (it != free.end() && it->size >= _bytes) {

                auto ptr = it->data;
                size_t remaining_size = it->size - _bytes;

                // If the block can be modified in place:
                if (remaining_size > 0) {
                    const_cast<block&>(*it).size = remaining_size;
                    const_cast<block&>(*it).data += _bytes;
                }
                else {
                    // Remove the block from the free list
                    free.erase(it);
                }

                return ptr;
            }
            else {
                result = nullptr;
            }

            return result;
        }

    protected:

        [[nodiscard]] HOT void* do_allocate(const size_t _bytes, const size_t _alignment) override {

            assert(_bytes > 0U && "Allocation size must be greater than zero.");
            assert((_alignment & (_alignment - 1U)) == 0U && "Alignment must be a power of two.");

            const size_t aligned_bytes = (_bytes + _alignment - 1U) & ~(_alignment - 1U);

            // Attempt to allocate from the stack block:
            if (m_stack_write + aligned_bytes <= s_stack_block_size) {

                auto* aligned_ptr = m_stack_block + ((m_stack_write + _alignment - 1U) & ~(_alignment - 1U));
                m_stack_write = static_cast<size_t>(aligned_ptr - m_stack_block) + aligned_bytes;

                return aligned_ptr;
            }

            // Attempt to find a free block, or create one otherwise:
            auto* aligned_ptr = allocate_from_free(_bytes, _alignment);
            if (aligned_ptr == nullptr) {
                expand(_bytes, _alignment);
                aligned_ptr = allocate_from_free(_bytes, _alignment);
            }

            if (aligned_ptr == nullptr) {
                throw std::bad_alloc();
            }

            return aligned_ptr;
        }

        HOT void do_deallocate(void* _p, size_t _bytes, size_t _alignment) override {

            assert(_p != nullptr && "Cannot deallocate null pointer.");

            if (((m_stack_write + (_bytes + _alignment - 1U)) & ~(_alignment - 1U)) > s_stack_block_size) {

                block freed_block(_bytes, _alignment, static_cast<uint8_t*>(_p));

                auto it = free.lower_bound(freed_block);

                // Merge with next block if adjacent:
                if (it != free.end() && freed_block.data + freed_block.size == it->data) {
                    freed_block.size += it->size;
                    freed_block.alignment = utils::max(it->alignment, freed_block.alignment);
                    it = free.erase(it);
                }

                // Merge with previous block if adjacent:
                if (it != free.begin()) {
                    --it;
                    if (it->data + it->size == freed_block.data) {
                        freed_block.data = it->data;
                        freed_block.size += it->size;
                        freed_block.alignment = utils::max(it->alignment, freed_block.alignment);
                        free.erase(it);
                    }
                }

                // Insert the merged block back into the free list:
                free.insert(freed_block);
            }
        }

        [[nodiscard]] bool do_is_equal(const memory_resource& _other) const noexcept override {
            return this == &_other;
        }

    public:

        explicit polytonic_pool() noexcept :
            m_stack_block(),
            initial_block_width(default_block_width),
            m_stack_write(0U),
            block_width(initial_block_width) {}

        explicit polytonic_pool(size_t _capacity) noexcept :
            m_stack_block(),
            initial_block_width(utils::min(_capacity, max_block_width)),
            m_stack_write(0U),
            block_width(initial_block_width
        ) {
            assert(_capacity >= 2U && "Capacity must be at least 2.");
        }

        ~polytonic_pool() override {
            cleanup();
        }

        void cleanup() {
            for (const auto& item : blocks) {
                ::operator delete(item.data, static_cast<std::align_val_t>(item.alignment));
            }
        }

        void reset() noexcept {

            m_stack_write = 0U;
            block_width = initial_block_width;

            try {
                free.clear();
                for (const auto& item : blocks) {
                    free.insert(item);
                }
            }
            catch (...) {
                  free.clear();
                blocks.clear();
            }
        }

        void release() noexcept {

            m_stack_write = 0U;
            block_width = initial_block_width;

            {
                cleanup();
                decltype(blocks) temp{};
                blocks = std::move(temp);
            }
            {
                decltype(free) temp{};
                free = std::move(temp);
            }
        }

    };

} //chdr

#endif //CHDR_POLYTONIC_POOL_HPP