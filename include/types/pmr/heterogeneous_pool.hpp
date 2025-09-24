/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_HETEROGENEOUS_POOL_HPP
#define CHDR_HETEROGENEOUS_POOL_HPP

/**
 * @file heterogeneous_pool.hpp
 */

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <new>
#include <utility>
#include <vector>

// ReSharper disable once CppUnusedIncludeDirective
#include <memory_resource> // NOLINT(*-include-cleaner)

// ReSharper disable once CppUnusedIncludeDirective
#include "../../utils/intrinsics.hpp" // NOLINT(*-include-cleaner)
#include "../../utils/utils.hpp"

namespace chdr {

    /**
     * @nosubgrouping
     * @class heterogeneous_pool
     * @brief A pooled memory resource for managing heterogeneous memory allocations.
     *
     * @details Provides efficient memory management by for situations where allocations may vary in size.
     *          It supports pre-allocated stack memory, dynamic memory expansion, and optional coalescing of
     *          memory blocks for reuse.
     *          Allocations are performed in blocks, reducing the overhead of frequent dynamic memory allocation.
     * 
     *          The primary features include:
     *          - Stack allocation for small data.
     *          - Allocation and deallocation of memory with alignment guarantees.
     *          - Block reuse through a free list.
     *          - Optional coalescing of adjacent free blocks for optimised memory reuse.
     *
     * @tparam              StackSize Size of the pool's stack buffer, in bytes. (optional, defaults to `4096`)
     * @tparam MaxStackAllocationSize Maximum size of a direct allocation to the stack buffer, in bytes. (optional)
     * @tparam       MaxHeapBlockSize Maximum size of a heap-allocated block, in bytes. (optional, defaults to `65536`)
     *
     * @tparam Coalescing Indicates whether the pool coalesces adjacent free memory blocks
     *                    for more efficient memory usage. (optional, defaults to `true`)
     *
     * @remarks Inherits from `std::pmr::memory_resource` to integrate with the
     *          PMR (Polymorphic Memory Resource) framework provided in the C++ Standard Library.
     */
    template <size_t StackSize = 4096U, size_t MaxStackAllocationSize = std::numeric_limits<size_t>::max(), size_t MaxHeapBlockSize = 65536U, bool Coalescing = true>
    class heterogeneous_pool final : public std::pmr::memory_resource {

        template <template <typename params_t> typename solver_t, typename params_t>
        friend class solvers::solver;

    private:

#if CHDR_DIAGNOSTICS == 1

        struct __chdr_internal_diagnostics {
            size_t  num_allocated { 0U };
            size_t peak_allocated { 0U };
        };

        __chdr_internal_diagnostics __diagnostic_data;

#endif //CHDR_DIAGNOSTICS == 1

        struct block final {

            size_t   size;
            size_t   alignment;
            uint8_t* data;

            [[nodiscard]] HOT constexpr block(size_t _size, size_t _alignment, uint8_t* _data) noexcept :
                size     (_size     ),
                alignment(_alignment),
                data     (_data     ) {}

            [[nodiscard]] friend HOT constexpr bool operator <(const block& _lhs, const block& _rhs) noexcept {
                return _lhs.size < _rhs.size;
            }
        };

        static constexpr size_t        s_stack_block_size { StackSize        };
        static constexpr size_t s_default_heap_block_size {  4096U           };
        static constexpr size_t     s_max_heap_block_size { MaxHeapBlockSize };

        // Fixed stack memory block:
        alignas(max_align_t) uint8_t m_stack_block[s_stack_block_size]; //NOLINT(*-avoid-c-arrays)

        size_t m_stack_write;         // Current write position for the stack block.
        size_t m_initial_block_width; // Width of the first allocated block.
        size_t m_block_width;         // Size of the current block.

        std::vector<block> m_blocks;
        std::map<size_t, block> m_free;

        HOT uint8_t* expand(size_t _bytes, size_t _alignment) noexcept {

            assert(_bytes > 0U && "Allocation size must be greater than zero.");
            assert((_alignment & (_alignment - 1U)) == 0U && "Alignment must be a power of two.");

            const auto allocate_size = utils::max(m_block_width, _bytes);

            uint8_t* result { nullptr };

            try {
                result = static_cast<uint8_t*>(operator new(allocate_size, static_cast<std::align_val_t>(_alignment)));

                m_blocks.emplace_back(
                    allocate_size,
                    _alignment,
                    result
                );

                // If the block is not entirely consumed, add the remaining space to the free list.
                if (const size_t remaining_size = allocate_size - _bytes;
                    remaining_size != 0U
                ) {

                    /*
                     * Attempt to insert new block into free list.
                     * Repeat with a smaller key upon collision.
                     * If insertion is impossible, throw.
                     */
                    bool success = false;
                    for (size_t i = 0U; i < remaining_size; ++i) {

                        if (m_free.try_emplace(remaining_size - i, remaining_size, _alignment, result + utils::max(_bytes, _alignment)).second) {
                            success = true;
                            break;
                        }
                    }

                    if (UNLIKELY(!success)) {
                        throw std::bad_alloc();
                    }
                }

                m_block_width = utils::min((m_block_width * 3U) / 2U, s_max_heap_block_size);
            }
            catch (...) {

                /* Catch any errors that occur during allocation. */

                // ReSharper disable once CppDFAConstantConditions
                if (result != nullptr) {
                    // ReSharper disable once CppDFAUnreachableCode
                    operator delete(result, static_cast<std::align_val_t>(_alignment));
                    result = nullptr;

                    m_blocks.pop_back();
                }
            }
            return result;
        }

        HOT uint8_t* allocate_from_free(size_t _bytes) {

            assert(_bytes > 0U && "Allocation size must be greater than zero.");

            uint8_t* result = nullptr;

            // Find the smallest free block that fits:
            auto it = m_free.lower_bound(_bytes);

            if (it != m_free.end() && it->second.size >= _bytes) {

                // Extract the matching block without performing a full tree rebalance:
                typename std::map<size_t, block>::node_type node = m_free.extract(it);

                result = node.mapped().data;

                // Consume block or adjust the remaining size:
                size_t remaining_size = node.mapped().size - _bytes;
                if (remaining_size != 0U) {

                    // Update with adjusted size:
                    node.key() = remaining_size;
                    node.mapped().size = remaining_size;
                    node.mapped().data += utils::max(_bytes, node.mapped().alignment);

                    m_free.insert(std::move(node)); // Reinsert (node is consumed otherwise).
                }
            }

            return result;
        }

        /**
         * @brief Frees all memory blocks managed by the memory pool.
         *
         * @details Iterates over all allocated blocks in the memory pool and deallocates
         *          their associated memory. This ensures that all memory resources used
         *          by the pool are released. The function is designed to handle memory
         *          in alignment-sensitive contexts, guaranteeing proper alignment for
         *          each block as it is deallocated.
         *
         *          It is intended to be called as part of the pool's release and
         *          destruction mechanisms.
         */
        void cleanup() noexcept {
            for (const auto& item : m_blocks) {
                operator delete(item.data, static_cast<std::align_val_t>(item.alignment));
            }
        }

    protected:

        /**
         * @brief Allocates memory with the specified size and alignment.
         *
         * @details Implements the allocation interface of `std::pmr::memory_resource`.
         *          This method attempts to allocate the requested memory in the following order:
         *          1. From the internal fixed-sized stack block if sufficient space is available.
         *          2. If the stack buffer does not have enough space to accommodate the allocation,
         *             it searches for a free block of memory that matches the allocation requirements.
         *          3. If no suitable free block is found, it attempts to expand the memory pool to fulfil the request.\n\n
         *          If none of the above options can satisfy the request, an exception of type `std::bad_alloc` is thrown.
         *          Alignment is guaranteed for both the stack and dynamic allocations as per the requested alignment.
         *
         * @param _bytes The size of the memory block to allocate, in bytes. Must be greater than `0`.
         * @param _alignment The alignment constraint for the start of the allocated memory block.
         *                   Must be a power of `2`.
         *
         * @pre _bytes must be greater than zero.
         * @pre _alignment must be a power of two.
         *
         * @post If the operation succeeds, the result will be a pointer to the aligned memory block of requested size.
         *       The user must not free this memory, as it belongs to the pool. Doing so will invoke undefined behaviour.
         *
         * @throws `std::bad_alloc` if the requested operation could not be completed.
         *
         * @return A pointer to the beginning of the allocated memory block.
         */
        [[nodiscard]] virtual HOT void* do_allocate(const size_t _bytes, const size_t _alignment) override {

            assert((_alignment & (_alignment - 1U)) == 0U && "Alignment must be a power of two.");

            if (_bytes == 0U) {
                return nullptr; // No-op. (See C++17 standard.)
            }
            else {
                const size_t aligned_bytes = (_bytes + _alignment - 1U) & ~(_alignment - 1U);

                uint8_t* aligned_ptr { nullptr };

                // Attempt to allocate from the stack block:
                if (aligned_bytes < MaxStackAllocationSize && m_stack_write + aligned_bytes <= s_stack_block_size) {

                    aligned_ptr = m_stack_block + ((m_stack_write + _alignment - 1U) & ~(_alignment - 1U));
                    m_stack_write = static_cast<size_t>(aligned_ptr - m_stack_block) + aligned_bytes;
                }

                if (aligned_ptr == nullptr) {

                    // Attempt to find a free block, or create one otherwise:
                    aligned_ptr = allocate_from_free(aligned_bytes);
                    if (UNLIKELY(aligned_ptr == nullptr)) {
                        aligned_ptr = expand(_bytes, _alignment);
                    }
                }

                // ReSharper disable once CppDFAConstantConditions
                if (UNLIKELY(aligned_ptr == nullptr)) {
                    // ReSharper disable once CppDFAUnreachableCode
                    throw std::bad_alloc();
                }

                PREFETCH(aligned_ptr, 1, 1);

#if CHDR_DIAGNOSTICS == 1

                __diagnostic_data. num_allocated += _bytes;
                __diagnostic_data.peak_allocated  = utils::max(__diagnostic_data.peak_allocated, __diagnostic_data.num_allocated);

#endif //CHDR_DIAGNOSTICS == 1

                return aligned_ptr;
            }
        }

        /**
         * @brief Deallocates memory and returns it to the pool for future use.
         *
         * @details Releases a previously allocated chunk of memory and stores it in the internal free list for reuse.
         *          If `Coalescing` is enabled, the freed memory block is coalesced with adjacent free blocks to reduce
         *          fragmentation.
         *
         * @param [in] _p Pointer to the memory block to be deallocated. Must not be null.
         * @param [in] _bytes Size of the memory block to be deallocated, in bytes.
         * @param [in] _alignment Alignment constraint for the start of the allocated memory block.
         *             Must be a power of two.
         *
         * @remarks The deallocation does not free the memory back to the system but recycles it
         *          internally for subsequent allocations.
         *
         * @pre _bytes must be greater than zero.
         * @pre _alignment must be a power of two.
         *
         * @pre Calling this function with a nullptr, or attempting to release memory not
         *      owned by the pool is undefined behaviour.

         * @post After this operation, the memory should not be used. Doing so is undefined behaviour.
         */
        virtual HOT void do_deallocate(void* _p, size_t _bytes, size_t _alignment) override {

            assert(_bytes > 0U && "Allocation size must be greater than zero.");
            assert((_alignment & (_alignment - 1U)) == 0U && "Alignment must be a power of two.");

            if (_bytes != 0U) {
                // No-op. (See C++17 standard.)
            }
            else {
                if (((m_stack_write + (_bytes + _alignment - 1U)) & ~(_alignment - 1U)) > s_stack_block_size) {

                    if constexpr (Coalescing) {

                        auto it = m_free.lower_bound(_bytes);

                        // Merge with next block if adjacent:
                        if (it != m_free.end() && _bytes + static_cast<uint8_t*>(_p) == it->second.data) {
                            _bytes    += it->second.size;
                            _alignment = utils::max(it->second.alignment, _alignment);

                            it = m_free.erase(it);
                        }

                        // Merge with previous block if adjacent:
                        if (it != m_free.begin()) {
                            --it;

                            if (it->second.data + it->second.size == _p) {
                                _p         = it->second.data;
                                _bytes    += it->second.size;
                                _alignment = utils::max(it->second.alignment, _alignment);

                                m_free.erase(it);
                            }
                        }
                    }

                    m_free.try_emplace(_bytes, _bytes, _alignment, static_cast<uint8_t*>(_p));
                }

#if CHDR_DIAGNOSTICS == 1

                __diagnostic_data.num_allocated = (_bytes > __diagnostic_data.num_allocated) ? 0U : (__diagnostic_data.num_allocated - _bytes);

#endif //CHDR_DIAGNOSTICS == 1
            }
        }

        /**
         * @brief Compares the equality of two memory resources.
         *
         * @details This method overrides the `do_is_equal` function of the `std::pmr::memory_resource` interface.
         *          It determines if the specified memory resource is the same as the current instance.
         *
         * @param _other [in] A reference to another `memory_resource` object to compare against.
         *
         * @returns `true` if the provided memory resource is the same instance as this one; `false` otherwise.
         */
        [[nodiscard]] virtual bool do_is_equal(const memory_resource& _other) const noexcept override {
            return this == &_other;
        }

    public:

        /**
         * @name Constructors
         * @{
         */

        /**
         * @brief Constructs a memory pool.
         *
         * @details Initialises a pooled memory resource.
         *          The constructor allows configuring the initial block width and overall capacity of the pool.
         *
         * @param _initial_block_width Defines the desired width for the memory blocks, in bytes.
         *                             If it exceeds the maximum block width, it will be clamped automatically.
         *                             Must be at least 2.
         *
         * @param _capacity Specifies the number of blocks the pool should initially reserve.
         *                  (optional, default value is `32U`).
         */
        explicit heterogeneous_pool(size_t _initial_block_width = s_default_heap_block_size, size_t _capacity = 32U) noexcept :
            m_stack_block        (),
            m_stack_write        (0U),
            m_initial_block_width(utils::min(_initial_block_width, s_max_heap_block_size)),
            m_block_width        (m_initial_block_width)
        {
            assert(_initial_block_width >= 2U && "Initial block width must be at least 2.");

            m_blocks.reserve(_capacity);
        }

        /**
         * @brief Destroys the object and releases all allocated memory.
         *
         * @details The destructor ensures that any memory managed by the pool
         *          is cleaned up properly by invoking the internal `cleanup()` method.
         *
         * @warning Manual destruction is not recommended and may result in undefined behaviour.
         *          Consider using `release()` or `reset()` instead.
         *
         * @see release()
         * @see reset()
         */
        virtual ~heterogeneous_pool() override {
            cleanup();
        }

        constexpr heterogeneous_pool           (const heterogeneous_pool&) = delete;
        constexpr heterogeneous_pool& operator=(const heterogeneous_pool&) = delete;

        [[nodiscard]]
#if __cplusplus >= 202002L
        constexpr
#endif
        heterogeneous_pool(heterogeneous_pool&& _other) noexcept :
            m_stack_block        (                            ),
            m_stack_write        (_other.m_stack_write        ),
            m_initial_block_width(_other.m_initial_block_width),
            m_block_width        (_other.m_block_width        ),
            m_blocks             (std::move(_other.m_blocks)  ),
            m_free               (std::move(_other.m_free  )  )
        {
            _other.release();
        }

#if __cplusplus >= 202002L
        constexpr
#endif
        heterogeneous_pool& operator=(heterogeneous_pool&& _other) noexcept {

            if (this != &_other) {

                cleanup();

                m_stack_write         = _other.m_stack_write;
                m_initial_block_width = _other.m_initial_block_width;
                m_block_width         = _other.m_block_width;
                m_blocks              = std::move(_other.m_blocks);
                m_free                = std::move(_other.m_free  );

                _other.release();
            }
            return *this;
        }

        /**
         * @}
         */

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
         * @post It is undefined behaviour to use memory previously allocated within the pool
         *       after calling of this function.
         *
         * @see release()
         */
        void reset() noexcept {

            m_stack_write = 0U;
            m_block_width = m_initial_block_width;

            try {
                m_free.clear();
                for (const auto& item : m_blocks) {
                    m_free.emplace(item.size, item);
                }
            }
            catch (...) {
                  m_free.clear();
                m_blocks.clear();
            }

#if CHDR_DIAGNOSTICS == 1
            __diagnostic_data. num_allocated = 0U;
            __diagnostic_data.peak_allocated = 0U;
#endif //CHDR_DIAGNOSTICS == 1
        }

        /**
         * @brief Releases all memory resources and resets the internal state of the memory pool.
         *
         * @details This method reinitialises the memory pool to its default state by resetting
         *          all internal bookkeeping measures.
         *          It also cleans up any allocated blocks and deallocates associated memory resources.
         *
         *          The operation is designed to deallocate and clear the entire pool, making it ready
         *          for reuse without the need to destroy the object.
         *
         * @warning After calling this method, all previously allocated memory from the pool
         *          is no longer accessible, and attempting to use such memory will result in undefined behaviour.
         *
         * @post It is undefined behaviour to use memory previously allocated within the pool
         *       after calling of this function.

         * @see reset()
         */
        void release() noexcept {

            m_stack_write = 0U;
            m_block_width = m_initial_block_width;

            {
                cleanup();
                decltype(m_blocks) temp{};
                m_blocks = std::move(temp);
            }
            {
                decltype(m_free) temp{};
                m_free = std::move(temp);
            }

#if CHDR_DIAGNOSTICS == 1
            __diagnostic_data. num_allocated = 0U;
            __diagnostic_data.peak_allocated = 0U;
#endif //CHDR_DIAGNOSTICS == 1
        }

#if CHDR_DIAGNOSTICS == 1
        constexpr const auto& __get_diagnostic_data() { return __diagnostic_data; };
#endif //CHDR_DIAGNOSTICS == 1

    };

} //chdr

#endif //CHDR_HETEROGENEOUS_POOL_HPP