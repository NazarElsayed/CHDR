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

            [[nodiscard]] HOT constexpr block(block&& other) noexcept = default;

#if __cplusplus > 202302L
            constexpr
#endif
            block& operator=(block&& other) noexcept = default;
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

                m_block_width = utils::min(m_block_width * 2U, s_max_block_width);
            }
            catch (...) {

                /* Catch any errors that occur during allocation. */

                if (result != nullptr) {
                    ::operator delete(result, static_cast<std::align_val_t>(_alignment));
                    result = nullptr;
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
            m_alignment          (0U),
            m_stack_write        (0U),
            m_initial_block_width(utils::min(_initial_block_width, s_max_block_width)),
            m_block_width        (m_initial_block_width)
        {
            assert(_capacity >= 2U && "Capacity must be at least 2.");

            m_blocks.reserve(_capacity);
              m_free.reserve(_capacity);
        }

        ~homogeneous_pool() override {
            cleanup();
        }

        constexpr homogeneous_pool           (const homogeneous_pool&) = delete;
        constexpr homogeneous_pool& operator=(const homogeneous_pool&) = delete;

        [[nodiscard]] constexpr homogeneous_pool(homogeneous_pool&& _other) noexcept :
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

        void cleanup() noexcept {
            for (const auto& item : m_blocks) {
                ::operator delete(item.data, static_cast<std::align_val_t>(m_alignment));
            }
        }

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