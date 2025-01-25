/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_GROWING_MONOTONIC_RESOURCE_HPP
#define CHDR_GROWING_MONOTONIC_RESOURCE_HPP

#include <cassert>
#include <cstddef>
#include <memory>
#include <vector>

#include "../../utils/utils.hpp"

namespace chdr {

    class monotonic_pool : public std::pmr::memory_resource {

        static constexpr size_t s_initial_heap_block_size {  4096U };
        static constexpr size_t     s_max_heap_block_size { 65536U };
        static constexpr size_t        s_stack_block_size {  4096U };

        struct block {
            std::unique_ptr<uint8_t[]> data;
            size_t size;
        };

        alignas(std::max_align_t) uint8_t m_stack_block[s_stack_block_size]; // Fixed stack memory block
        size_t m_stack_write{0U};                                            // Write pointer for stack block

        size_t m_current_block_size; // Size of the current block.
        size_t m_block_write;        // Current write position in the active block.
        size_t m_active_block_index; // Index of the current active block.

        std::vector<block> blocks;

        HOT void expand(size_t _size) {

            if (m_active_block_index + 1U < blocks.size()) { // Reuse an existing block:
                m_current_block_size = blocks[++m_active_block_index].size;
            }
            else {                                           // Allocate a new larger block:

                m_current_block_size = utils::max(
                    s_initial_heap_block_size,
                    utils::max(utils::min(m_current_block_size * 2U, s_max_heap_block_size), _size)
                );

                blocks.emplace_back(std::make_unique<uint8_t[]>(m_current_block_size), m_current_block_size);
                m_active_block_index = blocks.size() - 1U;
            }

            m_block_write = 0U; // Reset write head to beginning.
        }

    protected:

        [[nodiscard]] HOT void* do_allocate(const size_t _size, const size_t _alignment) override {

            // Try allocating from the stack first:
            if (m_stack_write + _size <= s_stack_block_size) {
                auto* aligned_ptr = reinterpret_cast<uint8_t*>(
                    (reinterpret_cast<uintptr_t>(m_stack_block + m_stack_write) + _alignment - 1U) & ~(_alignment - 1U)
                );

                auto new_position = static_cast<size_t>(aligned_ptr - m_stack_block) + _size;
                if (new_position <= s_stack_block_size) {
                    m_stack_write = new_position;
                    return aligned_ptr;
                }
            }

            // If stack block is exhausted, fall back to dynamic blocks:
            auto* aligned_ptr = reinterpret_cast<uint8_t*>(
                (reinterpret_cast<uintptr_t>(blocks[m_active_block_index].data.get() + m_block_write) + _alignment - 1U) & ~(_alignment - 1U)
            );

            if (aligned_ptr + _size > blocks[m_active_block_index].data.get() + blocks[m_active_block_index].size) {

                // Expand if the current block cannot fit the allocation.
                expand(_size + _alignment);
                return do_allocate(_size, _alignment);
            }

            // Update write position and return the aligned pointer
            m_block_write += static_cast<size_t>(aligned_ptr + _size - (blocks[m_active_block_index].data.get() + m_block_write));

            return aligned_ptr;
        }

        HOT void do_deallocate(void* /*__p*/, const size_t /*__bytes*/, size_t /*__alignment*/) override {
            // No-op.
        }

        [[nodiscard]] bool do_is_equal(const memory_resource& _other) const noexcept override {
            return this == &_other;
        }

    public:

        monotonic_pool() :
            m_current_block_size(s_initial_heap_block_size),
            m_block_write       (0U),
            m_active_block_index(0U)
        {
            expand(s_initial_heap_block_size); // Allocate the first dynamic block
        }

        constexpr monotonic_pool           (const monotonic_pool&) = delete;
        constexpr monotonic_pool& operator=(const monotonic_pool&) = delete;

        [[nodiscard]] constexpr monotonic_pool(monotonic_pool&&) noexcept = default;
    #if __cplusplus > 202302L
        constexpr
    #endif
        monotonic_pool& operator=(monotonic_pool&&) noexcept = default;

        [[nodiscard]] size_t allocated() const {

            size_t result = 0U;

            // Include stack usage.
            result += m_stack_write;

            // Include dynamic blocks.
            for (const auto& item : blocks) {
                result += item.size;
            }

            return result;
        }

        /**
         * @brief Resets the memory resource to reuse all previously allocated memory.
         */
        void reset() {
            m_stack_write        = 0U;
            m_active_block_index = 0U;
            m_block_write        = 0U;
        }

        /**
         * @brief Resets fully by clearing all blocks and starting fresh.
         */
        void release() {

            m_stack_write        = 0U;
            m_current_block_size = s_initial_heap_block_size;
            m_active_block_index = 0U;
            m_block_write        = 0U;

            decltype(blocks) temp{};
            blocks = std::move(temp);

            expand(s_initial_heap_block_size);
        }
    };

} //chdr

#endif // CHDR_GROWING_MONOTONIC_RESOURCE_HPP