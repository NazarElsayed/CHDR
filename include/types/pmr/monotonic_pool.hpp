/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_GROWING_MONOTONIC_RESOURCE_HPP
#define CHDR_GROWING_MONOTONIC_RESOURCE_HPP

/**
 * @file monotonic_pool.hpp
 */

#include <cassert>
#include <cstddef>
#include <memory>
#include <vector>

#include "../../utils/utils.hpp"

namespace chdr {

    /**
     * @nosubgrouping
     * @class monotonic_pool
     * @brief A pooled memory resource that manages memory blocks in a monotonic allocation scheme.
     *
     * @details Implements `std::pmr::memory_resource` and provides a pool-based memory allocation mechanism.
     *          It is designed for scenarios where multiple allocations are performed, but the memory is freed
     *          all at once rather than per individual allocation. Allocations are performed in blocks, reducing
     *          the overhead of frequent dynamic memory allocation.
     */
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

                    auto& recycled_block = m_blocks[++m_active_block_index];

                    m_current_block_size = recycled_block.size;
                    result = recycled_block.data;
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

        /**
         * @brief Frees all memory blocks managed by the memory pool.
         *
         * @details Iterates over all allocated blocks in the memory pool and deallocates
         *          their associated memory. This ensures that all memory resources used
         *          by the `monotonic_pool` are released. The function is designed to
         *          handle memory in alignment-sensitive contexts, guaranteeing proper
         *          alignment for each block as it is deallocated.
         *
         *          It is intended to be called as part of the pool's destruction or
         *          reset mechanisms.
         */
        void cleanup() noexcept {
            for (const auto& item : m_blocks) {
                ::operator delete(item.data, static_cast<std::align_val_t>(item.alignment));
            }
        }

    protected:

        /**
         * @brief Allocates memory with the specified size and alignment.
         *
         * @details Implements the allocation interface of `std::pmr::memory_resource` for the `monotonic_pool` class.
         *          This method attempts to allocate the requested memory in the following order:
         *          1. From the internal fixed-sized stack block (`m_stack_block`) if sufficient space is available.
         *          2. From the current active dynamic block if it has sufficient remaining capacity.
         *          3. By invoking the `expand` function, which creates a new block to fulfil the allocation request.\n\n
         *          If none of the above options can satisfy the request, an exception of type `std::bad_alloc` is thrown.
         *          Alignment is guaranteed for both the stack and dynamic allocations as per the requested alignment.
         *
         * @param _bytes The size, in bytes, of the memory to allocate. Must be greater than `0`.
         * @param _alignment The alignment requirement for the memory to allocate. Must be a power of `2`.
         *
         * @returns A pointer to the aligned memory block of the requested size. The caller must not manually free this memory.
         *
         * @throws std::bad_alloc If no memory block can be allocated for the requested size and alignment.
         */
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

        /**
         * @brief Does nothing. Deallocation is a no-op in monotonic allocation schemes.
         *
         * @details This method is overridden to comply with the `std::pmr::memory_resource` interface.
         *          As the `monotonic_pool` only supports monotonic allocation without individual
         *          deallocation, this implementation is a no-operation function. Any memory allocated
         *          by this resource will only be freed upon resetting or releasing the entire pool.
         *
         * @param __p [in] Pointer to the memory block to be deallocated. This parameter is ignored.
         * @param __bytes Size of the memory block to be deallocated, in bytes. This parameter is ignored.
         * @param __alignment Alignment of the memory block. This parameter is ignored.
         */
        HOT void do_deallocate(void* /*__p*/, const size_t /*__bytes*/, size_t /*__alignment*/) override {
            // No-op.
        }

        /**
         * @brief Compares the equality of two memory resource instances.
         *
         * @details This method overrides the `do_is_equal` function of the `std::pmr::memory_resource` interface.
         *          It determines if the specified memory resource is the same as the current instance.
         *
         * @param _other [in] A reference to another `memory_resource` object to compare against.
         *
         * @returns `true` if the provided memory resource is the same instance as this one; `false` otherwise.
         */
        [[nodiscard]] bool do_is_equal(const memory_resource& _other) const noexcept override {
            return this == &_other;
        }

    public:

        /**
         * @name Constructors
         * @{
         */

        /**
         * @brief Constructs the pool with default members.
         */
        monotonic_pool() noexcept :
            m_stack_block       (),
            m_current_block_size(s_initial_heap_block_size),
            m_stack_write       (0U),
            m_block_write       (0U),
            m_active_block_index(0U) {}

        /**
         * @brief Destroys the object and releases all allocated memory.
         *
         * @details The destructor ensures that any memory managed by the pool
         *          is cleaned up properly by invoking the internal `cleanup()` method.
         *
         * @warning Manually destruction is not recommended and may result in undefined behaviour.
         */
        [[deprecated("Manual destruction is not recommended and may result in undefined behaviour. "
                     "Use monotonic_pool::release() or monotonic_pool::reset() instead.")]]
        ~monotonic_pool() noexcept {
            cleanup();
        }

        constexpr monotonic_pool           (const monotonic_pool&) = delete;
        constexpr monotonic_pool& operator=(const monotonic_pool&) = delete;

        [[nodiscard]]
#if __cplusplus >= 202002L
        constexpr
#endif
        monotonic_pool(monotonic_pool&& _other) noexcept :
            m_stack_block       (                           ),
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

#if __cplusplus >= 202002L
        constexpr
#endif
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

        /**
         * @brief Resets the memory pool state to its initial configuration.
         *
         * @details Resets internal counters and indices, restoring the pool to a
         *          state as if no memory has been allocated. All blocks remain allocated,
         *          but any previously allocated data is effectively invalidated. This method
         *          does not deallocate or release memory already held by the pool.
         *
         * @warning After calling this method, all previously allocated memory from the pool
         *          should be deemed inaccessible.
         *
         * @see release()
         */
        void reset() noexcept {
            m_stack_write        = 0U;
            m_block_write        = 0U;
            m_active_block_index = 0U;
        }

        /**
         * @brief Releases all memory resources and resets the internal state of the memory pool.
         *
         * @details This method reinitialises the memory pool to its default state by resetting
         *          all internal bookkeeping measures, such as current block size, stack write position,
         *          block write position, and active block index. It also cleans up any allocated blocks
         *          and deallocates associated memory resources.
         *
         *          The operation is designed to deallocate and clear the entire pool, making it ready
         *          for reuse without the need to destroy the object.
         *
         * @warning After calling this method, all previously allocated memory from the pool
         *          is no longer accessible, and attempting to use such memory will result in undefined behaviour.
         *
         * @see reset()
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