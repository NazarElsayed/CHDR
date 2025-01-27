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

            [[nodiscard]] HOT constexpr block(block&& _other) noexcept = default;

#if __cplusplus > 202302L
            constexpr
#endif
            block& operator=(block&& _other) noexcept = default;

            [[nodiscard]] HOT constexpr bool operator < (const block& _other) const noexcept {
                return size < _other.size;
            }
        };

        static constexpr size_t s_initial_heap_block_size {  4096U };
        static constexpr size_t     s_max_heap_block_size { 65536U };
        static constexpr size_t        s_stack_block_size {  4096U };

        alignas(std::max_align_t) uint8_t m_stack_block[s_stack_block_size]; // Fixed stack memory block.

        size_t m_current_block_size; // Size of the current block.
        size_t m_stack_write;        // Current write position for stack block.
        size_t m_block_write;        // Current write position in the active block.
        size_t m_active_block_index; // Index of the current active block.

        std::vector<block> m_blocks;

        HOT uint8_t* expand(size_t _bytes, size_t _alignment) {

            uint8_t* result = nullptr;

            try {

                if (m_active_block_index + 1U < m_blocks.size()) { // Reuse an existing block:
                    m_current_block_size = m_blocks[++m_active_block_index].size;
                }
                else {                                             // Allocate a new, larger block:

                    m_current_block_size = utils::max(
                        s_initial_heap_block_size,
                        utils::max(utils::min((m_current_block_size * 3U) / 2U, s_max_heap_block_size), _bytes)
                    );

                    result = static_cast<uint8_t*>(::operator new(m_current_block_size, static_cast<std::align_val_t>(_alignment)));

                    m_blocks.emplace_back(
                        m_current_block_size,
                        _alignment,
                        result
                    );

                    m_active_block_index = m_blocks.size() - 1U;
                }

                m_block_write = 0U; // Reset write head to beginning.
            }
            catch (...) {

                /* Catch any errors that occur during allocation. */

                if (result != nullptr) {
                    ::operator delete(result, static_cast<std::align_val_t>(_alignment));
                    result = nullptr;

                    m_blocks.pop_back();
                }
            }

            return result;
        }

    protected:

        [[nodiscard]] HOT void* do_allocate(const size_t _bytes, const size_t _alignment) override {

            assert(_bytes > 0U && "Allocation size must be greater than zero.");

            uint8_t* aligned_ptr;

            // Try allocating from the stack first:
            if (m_stack_write + _bytes <= s_stack_block_size) {
                aligned_ptr = reinterpret_cast<uint8_t*>(
                    (reinterpret_cast<uintptr_t>(m_stack_block + m_stack_write) + _alignment - 1U) & ~(_alignment - 1U)
                );

                m_stack_write = static_cast<size_t>(aligned_ptr - m_stack_block) + _bytes;
                return aligned_ptr;
            }

            // If stack block is exhausted, fall back to dynamic blocks:
            if (!m_blocks.empty()) {

                aligned_ptr = reinterpret_cast<uint8_t*>(
                    (reinterpret_cast<uintptr_t>(m_blocks[m_active_block_index].data + m_block_write) + _alignment - 1U) & ~(_alignment - 1U)
                );

                // Invalidate if the current block cannot fit the allocation.
                if (aligned_ptr + _bytes > m_blocks[m_active_block_index].data + m_blocks[m_active_block_index].size) {
                    aligned_ptr = nullptr;
                }
            }
            else {
                aligned_ptr = nullptr;
            }

            // Expand if no valid candidate for allocation was found.
            if (aligned_ptr == nullptr) {
                aligned_ptr = expand(_bytes, _alignment);
            }

            // Update write position.
            m_block_write += static_cast<size_t>(aligned_ptr + _bytes - (m_blocks[m_active_block_index].data + m_block_write));

            if (aligned_ptr == nullptr) {
                throw std::bad_alloc();
            }

            return aligned_ptr;
        }

        HOT void do_deallocate(void* /*__p*/, const size_t /*__bytes*/, size_t /*__alignment*/) override {
            // No-op.
        }

        [[nodiscard]] bool do_is_equal(const memory_resource& _other) const noexcept override {
            return this == &_other;
        }

    public:

        monotonic_pool() noexcept :
            m_current_block_size(s_initial_heap_block_size),
            m_stack_write       (0U),
            m_block_write       (0U),
            m_active_block_index(0U) {}

        ~monotonic_pool() noexcept {
            cleanup();
        }

        constexpr monotonic_pool           (const monotonic_pool&) = delete;
        constexpr monotonic_pool& operator=(const monotonic_pool&) = delete;

        [[nodiscard]] constexpr monotonic_pool(monotonic_pool&& _other) noexcept :
            m_current_block_size(_other.m_current_block_size),
            m_stack_write       (_other.m_stack_write       ),
            m_block_write       (_other.m_block_write       ),
            m_active_block_index(_other.m_active_block_index),
            m_blocks            (std::move(_other.m_blocks) )
        {
            _other.m_stack_write        = 0U;
            _other.m_current_block_size = s_initial_heap_block_size;
            _other.m_active_block_index = 0U;
            _other.m_block_write        = 0U;
            _other.m_blocks.clear();
        }

        monotonic_pool& operator=(monotonic_pool&& _other) noexcept {

            if (this != &_other) {

                cleanup();

                m_current_block_size  = _other.m_current_block_size;
                m_stack_write         = _other.m_stack_write;
                m_block_write         = _other.m_block_write;
                m_active_block_index  = _other.m_active_block_index;
                m_blocks              = std::move(_other.m_blocks);

                _other.m_current_block_size = s_initial_heap_block_size;
                _other.m_stack_write        = 0U;
                _other.m_block_write        = 0U;
                _other.m_active_block_index = 0U;
                _other.m_blocks.clear();
            }
            return *this;
        }

        void cleanup() noexcept {
            for (const auto& item : m_blocks) {
                ::operator delete(item.data, static_cast<std::align_val_t>(item.alignment));
            }
        }

        /**
         * @brief Resets the memory resource to reuse all previously allocated memory.
         */
        void reset() noexcept {
            m_stack_write        = 0U;
            m_block_write        = 0U;
            m_active_block_index = 0U;
        }

        /**
         * @brief Resets fully by clearing all blocks and starting fresh.
         */
        void release() {

            m_current_block_size = s_initial_heap_block_size;
            m_stack_write        = 0U;
            m_block_write        = 0U;
            m_active_block_index = 0U;

            cleanup();
            decltype(m_blocks) temp{};
            m_blocks = std::move(temp);
        }
    };

} //chdr

#endif // CHDR_GROWING_MONOTONIC_RESOURCE_HPP