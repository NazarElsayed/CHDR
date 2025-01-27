/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_HETEROGENEOUS_POOL_HPP
#define CHDR_HETEROGENEOUS_POOL_HPP

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory_resource>
#include <utility>
#include <vector>

#include "../../utils/intrinsics.hpp" // NOLINT(*-include-cleaner)
#include "../../utils/utils.hpp"

namespace chdr {

    template <bool Coalescing = true>
    class heterogeneous_pool : public std::pmr::memory_resource {

    private:

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

        static constexpr size_t s_default_block_width {  4096U };
        static constexpr size_t     s_max_block_width { 65536U };
        static constexpr size_t    s_stack_block_size {  4096U };

        alignas(std::max_align_t) uint8_t m_stack_block[s_stack_block_size]; // NOLINT(*-avoid-c-arrays)

        size_t m_stack_write;
        size_t m_initial_block_width;
        size_t m_block_width;

        std::vector<block> m_blocks;
        std::map<size_t, block> m_free;

        HOT uint8_t* expand(size_t _bytes, size_t _alignment) noexcept {

            const auto allocate_size = utils::max(m_block_width, _bytes);

            uint8_t* result = nullptr;

            try {
                result = static_cast<uint8_t*>(::operator new(allocate_size, static_cast<std::align_val_t>(_alignment)));

                m_blocks.emplace_back(
                    allocate_size,
                    _alignment,
                    result
                );

                size_t remaining_size = allocate_size - _bytes;

                // If the block isn't entirely consumed, add remaining space to the free list.
                if (remaining_size != 0U) {

                    block new_block(
                        remaining_size, _alignment, result + utils::max(_bytes, _alignment)
                    );

                    /*
                     * Attempt to insert new block into free list.
                     * Repeat with a smaller key upon collision.
                     * If insertion is impossible, throw.
                     */
                    bool success = false;
                    for (size_t i = 0U; i < remaining_size; ++i) {
                        if (m_free.try_emplace(remaining_size - i, new_block).second) {
                            success = true;
                            break;
                        }
                    }

                    if (!success) {
                        throw std::bad_alloc();
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

        HOT uint8_t* allocate_from_free(size_t _bytes) {

            uint8_t* result;

            // Find the smallest free block that fits:
            auto it = m_free.lower_bound(_bytes);
            if (it != m_free.end() && it->second.size >= _bytes) {

                result = it->second.data;

                // Consume the block, or modify it in-place:
                auto remaining_size = it->second.size - _bytes;
                if (remaining_size != 0U) {

                    it->second.size  = remaining_size;
                    it->second.data += utils::max(_bytes, it->second.alignment);
                }
                else {
                    m_free.erase(it);
                }
            }
            else {
                result = nullptr;
            }

            return result;
        }

    protected:

        [[nodiscard]] HOT void* do_allocate(const size_t _bytes, const size_t _alignment) override {

            assert(_bytes > 0U && "Allocation size must be greater than zero.");
            assert((_alignment & (_alignment - 1U)) == 0U && "Alignment must be a power of two.");

            const size_t aligned_bytes = (_bytes + _alignment - 1U) & ~(_alignment - 1U);

            // Attempt to allocate from the stack block:
            if (m_stack_write + aligned_bytes <= s_stack_block_size) {

                auto* aligned_ptr = m_stack_block + ((m_stack_write + _alignment - 1U) & ~(_alignment - 1U));
                m_stack_write = static_cast<size_t>(aligned_ptr - m_stack_block) + aligned_bytes;

                return aligned_ptr;
            }

            // Attempt to find a free block, or create one otherwise:
            auto* aligned_ptr = allocate_from_free(aligned_bytes);
            if (aligned_ptr == nullptr) {
                aligned_ptr = expand(_bytes, _alignment);
            }

            if (aligned_ptr == nullptr) {
                throw std::bad_alloc();
            }

            return aligned_ptr;
        }

        HOT void do_deallocate(void* _p, size_t _bytes, size_t _alignment) override {

            assert(_p != nullptr && "Cannot deallocate null pointer.");

            if (((m_stack_write + (_bytes + _alignment - 1U)) & ~(_alignment - 1U)) > s_stack_block_size) {

                if constexpr (Coalescing) {

                    auto it = m_free.lower_bound(_bytes);

                    // Merge with next block if adjacent:
                    if (it != m_free.end() && static_cast<uint8_t*>(_p) + _bytes == it->second.data) {
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
        }

        [[nodiscard]] bool do_is_equal(const memory_resource& _other) const noexcept override {
            return this == &_other;
        }

    public:

        explicit heterogeneous_pool(size_t _initial_block_width = s_default_block_width, size_t _capacity = 32U) noexcept :
            m_stack_write(0U),
            m_initial_block_width(utils::min(_initial_block_width, s_max_block_width)),
            m_block_width(m_initial_block_width)
        {
            assert(_capacity >= 2U && "Capacity must be at least 2.");

            m_blocks.reserve(_capacity);
        }

        ~heterogeneous_pool() override {
            cleanup();
        }

        constexpr heterogeneous_pool           (const heterogeneous_pool&) = delete;
        constexpr heterogeneous_pool& operator=(const heterogeneous_pool&) = delete;

        [[nodiscard]] constexpr heterogeneous_pool(heterogeneous_pool&& _other) noexcept :
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

        heterogeneous_pool& operator=(heterogeneous_pool&& _other) noexcept {

            if (this != &_other) {

                cleanup();

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
                ::operator delete(item.data, static_cast<std::align_val_t>(item.alignment));
            }
        }

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
                decltype(m_free) temp{};
                m_free = std::move(temp);
            }
        }

    };

} //chdr

#endif //CHDR_HETEROGENEOUS_POOL_HPP