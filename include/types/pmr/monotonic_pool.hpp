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

        static constexpr size_t s_initial_block_size {  2048U };
        static constexpr size_t     s_max_block_size { 65536U };

        size_t m_current_block_size;    // Size of the current block.
        size_t m_block_write;           // Current write position in the active block.
        size_t m_active_block_index;    // Index of the current active block.

        std::vector<std::unique_ptr<char[]>> blocks; // NOLINT(*-avoid-c-arrays)
        std::vector<size_t> block_sizes;

        HOT void expand(const size_t& _size) {

            // If there are available preallocated blocks, overwrite them:
            if (m_active_block_index + 1U < blocks.size()) {
                m_current_block_size = block_sizes[++m_active_block_index];
            }
            else {
                // Otherwise, allocate a fresh block:
                m_current_block_size = utils::max(s_initial_block_size, utils::max(utils::min(m_current_block_size * 2U, s_max_block_size), _size));
                blocks.emplace_back(std::make_unique<char[]>(m_current_block_size)); // NOLINT(*-avoid-c-arrays)
                block_sizes.push_back(m_current_block_size);
                m_active_block_index = blocks.size() - 1U;
            }

            m_block_write = 0U; // Reset write head.
        }

    protected:

        [[nodiscard]] HOT void* do_allocate(const size_t _size, const size_t _alignment) override {

            // Ensure alignment and calculate aligned pointer:
            auto* aligned_ptr = reinterpret_cast<char*>(
                (reinterpret_cast<uintptr_t>(blocks[m_active_block_index].get() + m_block_write) + _alignment - 1U) & ~(_alignment - 1U)
            );

            // If insufficient space in the current block, expand and retry allocation:
            if (aligned_ptr + _size > blocks[m_active_block_index].get() + block_sizes[m_active_block_index]) {
                expand(_size + _alignment);
                return do_allocate(_size, _alignment);
            }

            // Update write position and return the aligned pointer:
            m_block_write += static_cast<size_t>(aligned_ptr + _size - (blocks[m_active_block_index].get() + m_block_write));
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
            m_current_block_size(s_initial_block_size),
            m_block_write       (0U),
            m_active_block_index(0U)
        {
            expand(s_initial_block_size); // Allocate first block.
        }

        constexpr monotonic_pool           (const monotonic_pool&) = delete;
        constexpr monotonic_pool& operator=(const monotonic_pool&) = delete;

        [[nodiscard]] constexpr monotonic_pool(monotonic_pool&&) noexcept = default;

#if __cplusplus > 202302L
        constexpr
#endif
        monotonic_pool& operator=(monotonic_pool&&) noexcept = default;
        
        size_t allocated() {

            size_t result { 0U };

            for (auto& size : block_sizes) {
                result += size;
            }

            return result;
        }

        /**
         * @brief Resets the memory resource to reuse all previously allocated memory.
         */
        void reset() {

            // Reset the index to the first block and prepare to overwrite.
            m_active_block_index = 0U;
            m_block_write        = 0U;
        }

        /**
         * @brief Resets fully by clearing all blocks and starting fresh.
         */
        void release() {

            // Reset block size, write position, and block index to initial values
            m_current_block_size = s_initial_block_size;
            m_active_block_index = 0U;
            m_block_write        = 0U;

            // Clear all existing blocks and their sizes.
            decltype(blocks) temp{};
            blocks      = std::move(temp);
            block_sizes = {};

            // Allocate a fresh initial block.
            expand(s_initial_block_size);
        }
    };

} //chdr

#endif // CHDR_GROWING_MONOTONIC_RESOURCE_HPP