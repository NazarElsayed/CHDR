/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_HOMOGENEOUS_POOL_HPP
#define CHDR_HOMOGENEOUS_POOL_HPP

/**
 * @file homogeneous_pool.hpp
 */

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <limits>
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
     * @class homogeneous_pool
     * @brief A pooled memory resource for managing homogeneous memory allocations.
     *
     * @details Provides efficient memory management by for situations where allocations
     *          are identical in size and alignment.
     *          It supports pre-allocated stack memory and dynamic memory expansion.
     *          Allocations are performed in blocks, reducing the overhead of frequent
     *          dynamic memory allocation.
     *
     *          The primary features include:
     *          - Support for stack allocation.
     *          - Allocation and deallocation of memory with alignment guarantees.
     *          - Block reuse through a free list.
     *
     * @tparam              StackSize Size of the pool's stack buffer, in bytes. (optional, defaults to `4096`)
     * @tparam MaxStackAllocationSize Maximum size of a direct allocation to the stack buffer, in bytes. (optional)
     * @tparam       MaxHeapBlockSize Maximum size of a heap-allocated block, in bytes. (optional, defaults to `65536`)
     *
     * @remarks Inherits from `std::pmr::memory_resource` to integrate with the
     *          PMR (Polymorphic Memory Resource) framework provided in the C++ Standard Library.
     */
    template <size_t StackSize = 4096U, size_t MaxStackAllocationSize = std::numeric_limits<size_t>::max(), size_t MaxHeapBlockSize = 65536U>
    class homogeneous_pool final : public std::pmr::memory_resource {

    private:

        struct block final {

            size_t   size;
            uint8_t* data;

            [[nodiscard]] HOT constexpr block(size_t _size, uint8_t* _data) noexcept :
                size(_size),
                data(_data) {}
        };

        static constexpr size_t        s_stack_block_size { StackSize        };
        static constexpr size_t s_default_heap_block_size {  4096U           };
        static constexpr size_t     s_max_heap_block_size { MaxHeapBlockSize };

        // Fixed stack memory block:
        alignas(max_align_t) uint8_t m_stack_block[s_stack_block_size]; //NOLINT(*-avoid-c-arrays)

        size_t m_alignment;           // Alignment of the allocated memory.
        size_t m_stack_write;         // Current write position for the stack block.
        size_t m_initial_block_width; // Width of the first allocated block.
        size_t m_block_width;         // Size of the current block.

        std::vector<block> m_blocks;
        std::vector<uint8_t*> m_free;

        HOT uint8_t* expand(size_t _bytes, size_t _alignment) noexcept {

            assert(_bytes > 0U && "Allocation size must be greater than zero.");
            assert((_alignment & (_alignment - 1U)) == 0U && "Alignment must be a power of two.");
            assert((m_alignment == 0U || _alignment == m_alignment) && "Alignment mismatch.");

            uint8_t* result { nullptr };

            try {
                const auto allocate_bytes = utils::max(m_block_width, _alignment);

                result = static_cast<uint8_t*>(operator new(allocate_bytes, static_cast<std::align_val_t>(_alignment)));

                m_blocks.emplace_back(
                    allocate_bytes,
                    result
                );

                const auto result_num = reinterpret_cast<uintptr_t>(result);

                // Alignment calculations:
                const auto aligned_chunk_bytes = (    _bytes + _alignment - 1U) & ~(_alignment - 1U);
                const auto aligned_base        = (result_num + _alignment - 1U) & ~(_alignment - 1U);

                // Divide into aligned chunks and distribute:
                if (const auto num_chunks = (allocate_bytes - (aligned_base - result_num)) / aligned_chunk_bytes;
                    num_chunks > 1U
                ) {
                    m_free.resize(m_free.size() + num_chunks - 1U, {});
                    auto currentSize = m_free.size();

                    IVDEP
                    for (size_t i = 1U; i < num_chunks; ++i) {
                        m_free[currentSize - i] = result + (aligned_chunk_bytes * (num_chunks - i));
                    }

                    if (!m_free.empty()) {
                        PREFETCH(m_free.back(), _MM_HINT_T0);
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

        HOT uint8_t* allocate_from_free() noexcept {

            uint8_t* result{};

            if (!m_free.empty()) {
                result = m_free.back();
                m_free.pop_back();
            }
            else {
                result = nullptr;
            }

            if (!m_free.empty()) {
                PREFETCH(m_free.back(), _MM_HINT_T0);
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
                operator delete(item.data, static_cast<std::align_val_t>(m_alignment));
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
         * @param [in] _bytes The size of the memory block to allocate, in bytes. Must be greater than `0`.
         * @param [in] _alignment The alignment constraint for the start of the allocated memory block.
         *                        Must be a power of `2`.
         *
         * @pre _bytes must be greater than zero.
         * @pre _alignment must be the same for all calls. It must be a power of two.
         *
         * @warning All subsequent calls to this function must use the same value for _bytes and _alignment.
         *          Not doing so is undefined behaviour.
         *
         * @post If the operation succeeds, the result will be a pointer to the aligned memory block of requested size.
         *       The user must not free this memory, as it belongs to the pool. Doing so will invoke undefined behaviour.
         *
         * @throws `std::bad_alloc` if the requested operation could not be completed.
         *
         * @return A pointer to the beginning of the allocated memory block.
         */
        [[nodiscard]] virtual HOT void* do_allocate(const size_t _bytes, const size_t _alignment) override {
            assert(_bytes > 0U && "Allocation size must be greater than zero.");
            assert((_alignment & (_alignment - 1U)) == 0U && "Alignment must be a power of two.");
            assert((m_alignment == 0U || _alignment == m_alignment) && "Alignment mismatch.");

            m_alignment = _alignment;

            uint8_t* aligned_ptr { nullptr };

            // Attempt to allocate from the stack block:
            if (m_stack_write < StackSize) {

                if (const size_t aligned_bytes = (_bytes + _alignment - 1U) & ~(_alignment - 1U);
                    aligned_bytes < MaxStackAllocationSize && m_stack_write + aligned_bytes <= s_stack_block_size
                ) {
                    aligned_ptr = m_stack_block + ((m_stack_write + _alignment - 1U) & ~(_alignment - 1U));
                    m_stack_write = static_cast<size_t>(aligned_ptr - m_stack_block) + aligned_bytes;
                }
            }

            if (aligned_ptr == nullptr) {

                // Attempt to find a free block, or create one otherwise:
                aligned_ptr = allocate_from_free();
                if (UNLIKELY(aligned_ptr == nullptr)) {
                    aligned_ptr = expand(_bytes, _alignment);
                }
            }

            // ReSharper disable once CppDFAConstantConditions
            if (UNLIKELY(aligned_ptr == nullptr)) {
                // ReSharper disable once CppDFAUnreachableCode
                throw std::bad_alloc();
            }

            return aligned_ptr;
        }

        /**
         * @brief Deallocates memory and returns it to the pool for future use.
         *
         * @details This method is used to release a previously allocated chunk of memory and store
         *          it in the internal free list for reuse.
         *
         * @param [in] _p Pointer to the memory block to be deallocated. Must not be null.
         * @param [in] _bytes Size of the memory block to be deallocated, in bytes. Must be greater than `0`
         *                    (currently unused).
         * @param [in] _alignment Alignment constraint for the start of the allocated memory block.
         *                        Must be a power of `2` (currently unused).
         *
         * @remarks The deallocation does not free the memory back to the system but recycles it
         *          internally for subsequent allocations.
         *
         * @pre _bytes must be greater than zero.
         * @pre _alignment must be the same for all calls. It must be a power of two.
         *
         * @pre The values for `_bytes` and `_alignment` must match those given when the memory was allocated.
         *      Otherwise, the operation may invoke undefined behaviour.
         *
         * @pre Calling this function with a nullptr, or attempting to release memory not
         *      owned by the pool is undefined behaviour.
         *
         * @post After this operation, the memory should not be used. Doing so is undefined behaviour.
         */
        virtual HOT void do_deallocate([[maybe_unused]] void* _p, [[maybe_unused]] const size_t _bytes, [[maybe_unused]] size_t _alignment) override {
            assert(_bytes > 0U && "Allocation size must be greater than zero.");
            assert((_alignment & (_alignment - 1U)) == 0U && "Alignment must be a power of two.");
            assert((m_alignment == 0U || _alignment == m_alignment) && "Alignment mismatch.");

            m_free.push_back(static_cast<uint8_t*>(_p));
        }

        /**
         * @brief Compares the equality of two memory resources.
         *
         * @details This method overrides the `do_is_equal` function of the `std::pmr::memory_resource` interface.
         *          It determines if the specified memory resource is the same as the current instance.
         *
         * @param [in] _other A reference to another `memory_resource` object to compare against.
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
        explicit homogeneous_pool(size_t _initial_block_width = s_default_heap_block_size, size_t _capacity = 32U) noexcept :
            m_stack_block        (),
            m_alignment          (0U),
            m_stack_write        (0U),
            m_initial_block_width(utils::min(_initial_block_width, s_max_heap_block_size)),
            m_block_width        (m_initial_block_width)
        {
            assert(_initial_block_width >= 2U && "Capacity must be at least 2.");

            m_blocks.reserve(_capacity);
              m_free.reserve(_capacity);
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
        virtual ~homogeneous_pool() override {
            cleanup();
        }

        constexpr homogeneous_pool           (const homogeneous_pool&) = delete;
        constexpr homogeneous_pool& operator=(const homogeneous_pool&) = delete;

        [[nodiscard]]
#if __cplusplus >= 202002L
        constexpr
#endif
        homogeneous_pool(homogeneous_pool&& _other) noexcept :
            m_stack_block        (                            ),
            m_alignment          (_other.m_alignment          ),
            m_stack_write        (_other.m_stack_write        ),
            m_initial_block_width(_other.m_initial_block_width),
            m_block_width        (_other.m_block_width        ),
            m_blocks             (std::move(_other.m_blocks)  ),
            m_free               (std::move(_other.m_free  )  )
        {
            _other.m_stack_write = 0U;
            _other.m_blocks.clear();
            _other.m_free.clear();
        }

#if __cplusplus >= 202002L
        constexpr
#endif
        homogeneous_pool& operator=(homogeneous_pool&& _other) noexcept {

            if (this != &_other) {

                cleanup();

                m_alignment           = _other.m_alignment;
                m_stack_write         = _other.m_stack_write;
                m_initial_block_width = _other.m_initial_block_width;
                m_block_width         = _other.m_block_width;
                m_blocks              = std::move(_other.m_blocks);
                m_free                = std::move(_other.m_free  );

                _other.m_stack_write = 0U;
                _other.m_blocks.clear();
                _other.m_free.clear();
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
        HOT void reset() noexcept {

            m_stack_write = 0U;
            m_block_width = m_initial_block_width;

            try {
                m_free.clear();

                // Reuse memory from existing blocks to repopulate `free`:
                for (auto& [size, data] : m_blocks) {

                    // Divide the block into chunks; all chunks are guaranteed aligned.
                    const auto chunk_size = size / m_block_width;

                    const auto current_size = m_free.size();
                    m_free.resize(current_size + chunk_size);

                    IVDEP
                    for (size_t j = 0U; j < chunk_size; ++j) {
                        m_free[j + current_size] = data + j * m_block_width;
                    }
                }

                if (!m_free.empty()) {
                    PREFETCH(m_free.back(), _MM_HINT_T0);
                }
            }
            catch (...) {
                m_blocks.clear();
                  m_free.clear();
            }
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
                m_free = {};
            }
        }
    };

} //chdr

#endif // CHDR_HOMOGENEOUS_POOL_HPP