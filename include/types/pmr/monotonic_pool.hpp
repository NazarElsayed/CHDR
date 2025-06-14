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
#include <cstdint>
#include <limits>
#include <memory>
#include <new>
#include <vector>

// ReSharper disable once CppUnusedIncludeDirective
#include <memory_resource> // NOLINT(*-include-cleaner)

#include "../../utils/utils.hpp"

// ReSharper disable once CppUnusedIncludeDirective
#include "../../utils/intrinsics.hpp" // NOLINT(*-include-cleaner)

namespace chdr {

    /**
     * @nosubgrouping
     * @class monotonic_pool
     * @brief A pooled memory resource that manages memory blocks in a monotonic allocation scheme.
     *
     * @details Designed for scenarios where multiple allocations are performed, but the memory is freed all
     *          at once rather than per individual allocation. Allocations are performed in blocks, reducing
     *          the overhead of frequent dynamic memory allocation.
     *
     * @tparam              StackSize Size of the pool's stack buffer, in bytes. (optional, defaults to `4096`)
     * @tparam MaxStackAllocationSize Maximum size of a direct allocation to the stack buffer, in bytes. (optional)
     * @tparam       MaxHeapBlockSize Maximum size of a heap-allocated block, in bytes. (optional, defaults to `65536`)
     *
     * @remarks Inherits from `std::pmr::memory_resource` to integrate with the
     *          PMR (Polymorphic Memory Resource) framework provided in the C++ Standard Library.
     */
    template <size_t StackSize = 4096U, size_t MaxStackAllocationSize = std::numeric_limits<size_t>::max(), size_t MaxHeapBlockSize = 65536U>
    class monotonic_pool final : public std::pmr::memory_resource {

        template <template <typename params_t> typename solver_t, typename params_t>
        friend class solvers::solver;

    private:

#if CHDR_DIAGNOSTICS == 1

        size_t  num_allocated { 0U };
        size_t peak_allocated { 0U };

#endif

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
        size_t m_block_write;         // Current write position in the active block.
        size_t m_active_block_index;  // Index of the current active block.
        size_t m_initial_block_width; // Width of the first allocated block.
        size_t m_block_width;         // Size of the current block.

        std::vector<block> m_blocks;

        HOT uint8_t* expand(size_t _bytes, size_t _alignment) {

            assert(_bytes > 0U && "Allocation size must be greater than zero.");
            assert((_alignment & (_alignment - 1U)) == 0U && "Alignment must be a power of two.");

            uint8_t* result { nullptr };

            try {

                if (m_active_block_index + 1U < m_blocks.size()) { // Reuse an existing block:

                    auto& recycled_block = m_blocks[++m_active_block_index];

                    m_block_width = recycled_block.size;
                    result = recycled_block.data;
                }
                else { // Allocate a new, larger block:

                    m_block_width = utils::max(
                        m_initial_block_width,
                        utils::max(utils::min((m_block_width * 3U) / 2U, s_max_heap_block_size), _bytes)
                    );

                    result = static_cast<uint8_t*>(operator new(m_block_width, static_cast<std::align_val_t>(_alignment)));

                    m_blocks.emplace_back(
                        m_block_width,
                        _alignment,
                        result
                    );

                    m_active_block_index = m_blocks.size() - 1U;
                }

                m_block_write = 0U; // Reset write head to beginning.
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
         *          2. From the current active dynamic block if it has sufficient remaining capacity.
         *          3. By invoking the `expand` function, which creates a new block to fulfil the allocation request.\n\n
         *          If none of the above options can satisfy the request, an exception of type `std::bad_alloc` is thrown.
         *          Alignment is guaranteed for both the stack and dynamic allocations as per the requested alignment.
         *
         * @param _bytes The size, in bytes, of the memory to allocate. Must be greater than `0`.
         * @param _alignment The alignment requirement for the memory to allocate. Must be a power of `2`.
         *
         * @pre _bytes must be greater than zero.
         * @pre _alignment must be a power of two.
         *
         * @post If the operation succeeds, the result will be a pointer to the aligned memory block of requested size.
         *       The user must not free this memory, as it belongs to the pool. Doing so will invoke undefined behaviour.
         *
         * @throws `std::bad_alloc` if the requested operation could not be completed.
         *
         * @returns A pointer to the aligned memory block of the requested size. The caller must not manually free this memory.
         */
        [[nodiscard]] virtual HOT void* do_allocate(const size_t _bytes, const size_t _alignment) override {

            assert(_bytes > 0U && "Allocation size must be greater than zero.");
            assert((_alignment & (_alignment - 1U)) == 0U && "Alignment must be a power of two.");

            uint8_t* aligned_ptr { nullptr };

            // Attempt to allocate from the stack block:
            if (m_stack_write < StackSize) {

                if (const size_t aligned_bytes = (_bytes + _alignment - 1U) & ~(_alignment - 1U);
                    aligned_bytes < MaxStackAllocationSize &&
                    m_stack_write + aligned_bytes <= s_stack_block_size
                ) {
                    aligned_ptr = m_stack_block + ((m_stack_write + _alignment - 1U) & ~(_alignment - 1U));
                    m_stack_write = static_cast<size_t>(aligned_ptr - m_stack_block) + aligned_bytes;
                }
            }

            if (aligned_ptr == nullptr) {

                // If the stack block is exhausted, fall back to dynamic blocks:
                if (LIKELY(!m_blocks.empty())) {

                    auto*  raw_ptr = static_cast<void*>(static_cast<uint8_t*>(m_blocks[m_active_block_index].data) + m_block_write);
                    size_t space   = m_blocks[m_active_block_index].size - m_block_write;

                    aligned_ptr = static_cast<uint8_t*>(std::align(_alignment, _bytes, raw_ptr, space));
                    if (aligned_ptr != nullptr) {
                        m_block_write = static_cast<size_t>(static_cast<uint8_t*>(aligned_ptr) - static_cast<uint8_t*>(m_blocks[m_active_block_index].data)) + _bytes;
                    }
                }
                else {
                    aligned_ptr = nullptr;
                }

                // Expand if no valid candidate for allocation was found.
                if (UNLIKELY(aligned_ptr == nullptr)) {
                    aligned_ptr = expand(_bytes, _alignment);
                }

                // Update write position.
                m_block_write += static_cast<size_t>(aligned_ptr + _bytes - (m_blocks[m_active_block_index].data + m_block_write));
            }

            // ReSharper disable once CppDFAConstantConditions
            if (UNLIKELY(aligned_ptr == nullptr)) {
                // ReSharper disable once CppDFAUnreachableCode
                throw std::bad_alloc();
            }

            PREFETCH(aligned_ptr, _MM_HINT_T0);

#if CHDR_DIAGNOSTICS == 1

             num_allocated += _bytes;
            peak_allocated  = utils::max(peak_allocated, num_allocated);

#endif

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
         * @param [in] _p Pointer to the memory block to be deallocated. Must not be null. Currently unused.
         * @param [in] _bytes Size of the memory block to be deallocated, in bytes. Currently unused.
         * @param [in] _alignment Alignment constraint for the start of the allocated memory block.
         *             Must be a power of two. Currently unused.
         *
         * @post This function is a no-op and calling it does nothing, as the pool only supports deallocating all
         *       memory at once. Consider using `reset()` or `release()` instead.
         *
         * @see release()
         * @see reset()
         */
        virtual HOT void do_deallocate([[maybe_unused]] void* _p, [[maybe_unused]] const size_t _bytes, [[maybe_unused]] size_t _alignment) override {
            // No-op.
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
         * @details Initialises a pooled memory resource.
         * @param _initial_block_width Width of the first dynamically-allocated block. (optional)
         */
        monotonic_pool(size_t _initial_block_width = s_default_heap_block_size) noexcept :
            m_stack_block        (),
            m_stack_write        (0U),
            m_block_write        (0U),
            m_active_block_index (0U),
            m_initial_block_width(utils::min(_initial_block_width, s_max_heap_block_size)),
            m_block_width        (m_initial_block_width) {}

        /**
         * @brief Destroys the object and releases all allocated memory.
         *
         * @details The destructor ensures that any memory managed by the pool
         *          is cleaned up properly by invoking the internal `cleanup()` method.
         *
         * @warning Manual destruction is not recommended and may result in undefined behaviour.
         *          Consider using `reset()` or `release()` instead.
         *
         * @see release()
         * @see reset()
         */
        virtual ~monotonic_pool() noexcept override {
            cleanup();
        }

        constexpr monotonic_pool           (const monotonic_pool&) = delete;
        constexpr monotonic_pool& operator=(const monotonic_pool&) = delete;

        [[nodiscard]]
#if __cplusplus >= 202002L
        constexpr
#endif
        monotonic_pool(monotonic_pool&& _other) noexcept :
            m_stack_block        (),
            m_stack_write        (_other.m_stack_write        ),
            m_block_write        (_other.m_block_write        ),
            m_active_block_index (_other.m_active_block_index ),
            m_initial_block_width(_other.m_initial_block_width),
            m_block_width        (_other.m_block_width        ),
            m_blocks             (std::move(_other.m_blocks)  )
        {
            _other.release();
        }

#if __cplusplus >= 202002L
        constexpr
#endif
        monotonic_pool& operator=(monotonic_pool&& _other) noexcept {

            if (this != &_other) {

                cleanup();

                m_stack_write         = _other.m_stack_write;
                m_block_write         = _other.m_block_write;
                m_active_block_index  = _other.m_active_block_index;
                m_initial_block_width = _other.m_initial_block_width;
                m_block_width         = _other.m_block_width;
                m_blocks              = std::move(_other.m_blocks);

                _other.release();
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
         * @post It is undefined behaviour to use memory previously allocated within the pool
         *       after calling of this function.
         *
         * @see release()
         */
        void reset() noexcept {
            m_stack_write        = 0U;
            m_block_write        = 0U;
            m_active_block_index = 0U;

#if CHDR_DIAGNOSTICS == 1

             num_allocated = 0U;
            peak_allocated = 0U;

#endif
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
         *
         * @see reset()
         */
        void release() {

            m_block_width        = m_initial_block_width;
            m_stack_write        = 0U;
            m_block_write        = 0U;
            m_active_block_index = 0U;

            cleanup();
            decltype(m_blocks) temp{};
            m_blocks = std::move(temp);

#if CHDR_DIAGNOSTICS == 1

             num_allocated = 0U;
            peak_allocated = 0U;

#endif
        }
    };

} //chdr

#endif // CHDR_GROWING_MONOTONIC_RESOURCE_HPP