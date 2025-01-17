/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_BUMP_ALLOCATOR_HPP
#define CHDR_BUMP_ALLOCATOR_HPP

#include <cassert>
#include <cstddef>
#include <memory>
#include <vector>

namespace chdr {

    class growing_monotonic_resource : public std::pmr::memory_resource {

        static constexpr size_t s_initial_block_size {  1024U };
        static constexpr size_t     s_max_block_size { 65536U };

        size_t m_current_block_size;    // Size of the current block.
        size_t m_block_write;           // Current write position in the active block.
        size_t m_active_block_index;    // Index of the current active block.

        std::vector<std::unique_ptr<char[]>> blocks;    // List of allocated blocks.
        std::vector<size_t> block_sizes;                // Sizes of the allocated blocks.

        void expand(const size_t& _size) {

            // If there are available preallocated blocks, overwrite them:
            if (m_active_block_index + 1U < blocks.size()) {
                m_current_block_size = block_sizes[++m_active_block_index];
            }
            else {
                // Otherwise, allocate a fresh block:
                m_current_block_size = utils::max(s_initial_block_size, utils::max(utils::min(m_current_block_size * 2U, s_max_block_size), _size));
                blocks.emplace_back(std::make_unique<char[]>(m_current_block_size));
                block_sizes.push_back(m_current_block_size);
                m_active_block_index = blocks.size() - 1U;
            }

            m_block_write = 0U; // Reset write head.
        }

    protected:

        void* do_allocate(const size_t _size, const size_t _alignment) override {

            // Ensure alignment:
            char* current     = blocks[m_active_block_index].get() + m_block_write;
            size_t remaining  = block_sizes[m_active_block_index]  - m_block_write;
            void* aligned_ptr = std::align(_alignment, _size, reinterpret_cast<void*&>(current), remaining);

            // Handle insufficient space in the current block:
            if (aligned_ptr == nullptr) {
                expand(_size + _alignment);
                return do_allocate(_size, _alignment);
            }

            // Update the write position and return the pointer:
            m_block_write = static_cast<size_t>(current + _size - blocks[m_active_block_index].get());
            return aligned_ptr;
        }

        void do_deallocate(void*, const size_t, size_t) override {
            // No-op, as memory within a monotonic resource is not individually freed.
        }

        bool do_is_equal(const memory_resource& _other) const noexcept override {
            return this == &_other;
        }

    public:
        growing_monotonic_resource() : m_current_block_size(s_initial_block_size),
            m_block_write       (0U),
            m_active_block_index(0U)
        {
            expand(s_initial_block_size); // Allocate first block.
        }

        /**
         * @brief Resets the memory resource to reuse all previously allocated memory.
         */
        void release() {
            // Reset the index to the first block and prepare to overwrite.
            m_active_block_index = 0U;
            m_block_write        = 0U;
        }

        /**
         * @brief Resets fully by clearing all blocks and starting fresh.
         */
        void reset() {

            // Clear all existing blocks and their sizes.
                 blocks.clear();
            block_sizes.clear();

            // Reset block size, write position, and block index to initial values
            m_current_block_size = s_initial_block_size;
            m_active_block_index = 0U;
            m_block_write        = 0U;

            // Allocate a fresh initial block.
            expand(s_initial_block_size);
        }
    };

} //chdr

#endif // CHDR_BUMP_ALLOCATOR_HPP