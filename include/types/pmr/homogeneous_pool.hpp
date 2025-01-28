/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_HOMOGENEOUS_POOL_HPP
#define CHDR_HOMOGENEOUS_POOL_HPP

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <memory_resource>
#include <utility>
#include <vector>
#include <algorithm>

#include "../../utils/intrinsics.hpp" // NOLINT(*-include-cleaner)
#include "../../utils/utils.hpp"

namespace chdr {

    class homogeneous_pool : public std::pmr::memory_resource {
    
    private:

        struct block final {

            size_t   size;
            uint8_t* data;

            [[nodiscard]] HOT constexpr block(size_t _size, uint8_t* _data) noexcept :
                size(_size),
                data(_data) {}

            ~block() = default;

            [[nodiscard]] HOT constexpr block           (const block&) = default;
                          HOT constexpr block& operator=(const block&) = default;

            [[nodiscard]] HOT constexpr block(block&& _other) noexcept = default;

#if __cplusplus > 202302L
            constexpr
#endif
            block& operator=(block&& _other) noexcept = default;
        };

        static constexpr size_t s_default_block_width {  4096U };
        static constexpr size_t     s_max_block_width { 65536U };
        static constexpr size_t    s_stack_block_size {  4096U };

        alignas(std::max_align_t) uint8_t m_stack_block[s_stack_block_size]; // NOLINT(*-avoid-c-arrays)

        size_t m_alignment;
        size_t m_stack_write;
        size_t m_initial_block_width;
        size_t m_block_width;

        std::vector<block> m_blocks;
        std::vector<uint8_t*> m_free;

        HOT uint8_t* expand(size_t _size, size_t _alignment) noexcept {

            uint8_t* result = nullptr;

            try {
                const auto allocate_size = utils::max(m_block_width, _alignment);

                result = static_cast<uint8_t*>(::operator new(allocate_size, static_cast<std::align_val_t>(_alignment)));

                m_blocks.emplace_back(
                    allocate_size,
                    result
                );

                const auto result_num = reinterpret_cast<uintptr_t>(result);

                // Alignment calculations:
                const auto aligned_chunk_size = (     _size + _alignment - 1U) & ~(_alignment - 1U);
                const auto aligned_base       = (result_num + _alignment - 1U) & ~(_alignment - 1U);

                // Divide into aligned chunks:
                const auto num_chunks = (allocate_size - (aligned_base - result_num)) / aligned_chunk_size;

                // Distribute free chunks:
                if (num_chunks > 1U) {
                    m_free.reserve(m_free.size() + num_chunks - 1U);

                    IVDEP
                    for (size_t i = 1U; i < num_chunks; ++i) {
                        m_free.emplace_back(result + (aligned_chunk_size * i));
                    }
                }

                m_block_width = utils::min((m_block_width * 3U) / 2U, s_max_block_width);
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

        HOT uint8_t* allocate_from_free() noexcept {

            uint8_t* result;

            if (!m_free.empty()) {
                result = m_free.back();
                m_free.pop_back();
            }
            else {
                result = nullptr;
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
                ::operator delete(item.data, static_cast<std::align_val_t>(m_alignment));
            }
        }

    protected:

        [[nodiscard]] HOT void* do_allocate(const size_t _bytes, const size_t _alignment) override {
            assert(_bytes > 0U && "Allocation size must be greater than zero.");

            m_alignment = _alignment;

            // Attempt to allocate from the stack block:
			{
			    const size_t aligned_bytes = (_bytes + _alignment - 1U) & ~(_alignment - 1U);

                if (m_stack_write + aligned_bytes <= s_stack_block_size) {

                    auto* aligned_ptr = m_stack_block + ((m_stack_write + _alignment - 1U) & ~(_alignment - 1U));
                    m_stack_write = static_cast<size_t>(aligned_ptr - m_stack_block) + aligned_bytes;

                    return aligned_ptr;
                }
            }

            // Attempt to find a free block, or create one otherwise:
            auto* aligned_ptr = allocate_from_free();
            if (aligned_ptr == nullptr) {
                aligned_ptr = expand(_bytes, _alignment);
            }

            if (aligned_ptr == nullptr) {
                throw std::bad_alloc();
            }

            return aligned_ptr;
        }

        HOT void do_deallocate(void* _p, size_t, size_t) override {
            assert(_p != nullptr && "Cannot deallocate null pointer.");

            m_free.push_back(static_cast<uint8_t*>(_p));
        }

        [[nodiscard]] bool do_is_equal(const memory_resource& _other) const noexcept override {
            return this == &_other;
        }

    public:

        explicit homogeneous_pool(size_t _initial_block_width = s_default_block_width, size_t _capacity = 32U) noexcept :
            m_stack_block        (),
            m_alignment          (0U),
            m_stack_write        (0U),
            m_initial_block_width(utils::min(_initial_block_width, s_max_block_width)),
            m_block_width        (m_initial_block_width)
        {
            assert(_capacity >= 2U && "Capacity must be at least 2.");

            m_blocks.reserve(_capacity);
              m_free.reserve(_capacity);
        }

        /**
         * @brief Destroys the object and releases all allocated memory.
         *
         * @details The destructor ensures that any memory managed by the pool
         *          is cleaned up properly by invoking the internal `cleanup()` method.
         *
         * @warning Manually destruction is not recommended and may result in undefined behaviour.
         */
        ~homogeneous_pool() override {
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
            _other.m_stack_write = 0;
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

} // namespace chdr

#endif // CHDR_HOMOGENEOUS_POOL_HPP