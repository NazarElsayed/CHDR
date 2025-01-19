/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

// ReSharper disable CppInconsistentNaming

#ifndef CHDR_POLYTONIC_POOL_HPP
#define CHDR_POLYTONIC_POOL_HPP

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "../../utils/intrinsics.hpp" // NOLINT(*-include-cleaner)
#include "../../utils/utils.hpp"

namespace chdr {

    class polytonic_pool : public std::pmr::memory_resource {

    private:

        static constexpr size_t default_block_width {  2048U };
        static constexpr size_t     max_block_width { 65536U };

        size_t initial_block_width;
        size_t block_width;

        std::vector<std::unique_ptr<char[]>> blocks; // NOLINT(*-avoid-c-arrays)
        std::vector<size_t>                  block_sizes;
        std::vector<void*>                   free;

        void* expand(const size_t& _size, const size_t& _alignment) {

            // Compute the properly aligned block size for this chunk.
            const auto aligned_chunk_size = (_size + _alignment - 1U) & ~(_alignment - 1U);

            // Allocate memory:
            const auto allocate_size = utils::max(block_width, aligned_chunk_size);
            blocks.emplace_back(std::make_unique<char[]>(allocate_size)); // NOLINT(*-avoid-c-arrays)
            block_sizes.emplace_back(allocate_size);
            auto* new_block = blocks.back().get();

            // Pre-align the base pointer of the new block.
            const auto aligned_base = (reinterpret_cast<uintptr_t>(new_block) + _alignment - 1U) & ~(_alignment - 1U);

            // Divide the block into aligned chunks:
            const auto num_chunks = (allocate_size - (aligned_base - reinterpret_cast<uintptr_t>(new_block))) / aligned_chunk_size;

            free.reserve(free.size() + num_chunks - 1U);

            IVDEP
            for (size_t i = 0U; i != num_chunks - 1U; ++i) {
                free.emplace_back(reinterpret_cast<void*>(aligned_base + (i * aligned_chunk_size)));
            }

            block_width = utils::min(block_width * 2U, max_block_width);
            return reinterpret_cast<void*>(aligned_base + ((num_chunks - 1U) * aligned_chunk_size));
        }

    protected:

        [[nodiscard]] void* do_allocate(const size_t _size, const size_t _alignment) override {
            assert(_size > 0U && "Allocation size must be greater than zero.");

            // Check for available free chunks:
            if (!free.empty()) {
                auto* chunk = free.back();
                free.pop_back();
                return chunk; // All chunks are pre-aligned.
            }

            // Expand pool otherwise.
            return expand(_size, _alignment);
        }

        void do_deallocate(void* _p, size_t, size_t) override {
            assert(_p != nullptr && "Cannot deallocate null pointer.");
            free.push_back(_p);
        }

        [[nodiscard]] bool do_is_equal(const memory_resource& _other) const noexcept override {
            return this == &_other;
        }

    public:

        explicit polytonic_pool() noexcept :
            initial_block_width(default_block_width),
                    block_width(initial_block_width) {}

        explicit polytonic_pool(const size_t& _capacity) noexcept :
            initial_block_width(utils::min(_capacity, max_block_width)),
                    block_width(initial_block_width)
        {
            assert(_capacity >= 2U && "Capacity must be at least 2.");
        }

        constexpr polytonic_pool           (const polytonic_pool&) = delete;
        constexpr polytonic_pool& operator=(const polytonic_pool&) = delete;

        [[nodiscard]] constexpr polytonic_pool(polytonic_pool&&) noexcept = default;

#if __cplusplus > 202302L
        constexpr
#endif
        polytonic_pool& operator=(polytonic_pool&&) noexcept = default;

        size_t allocated() {

            size_t result { 0U };

            for (auto& size : block_sizes) {
                result += size;
            }

            return result;
        }
        
        void reset() noexcept {

            block_width = initial_block_width;

            try {
                free.clear();

                // Reuse memory from existing blocks to repopulate `free`:
                for (size_t i = 0U; i < blocks.size(); ++i) {

                    auto*      block      = blocks[i].get();
                    const auto block_size = block_sizes[i];

                    // Divide the block into chunks; all chunks are guaranteed aligned.
                    const auto chunk_size = block_size / block_width;

                    const auto current_size = free.size();
                    free.resize(current_size + chunk_size);

                    IVDEP
                    for (size_t j = 0U; j < chunk_size; ++j) {
                        free[j + current_size] = block + j * block_width;
                    }
                }
            }
            catch (...) {
                     blocks.clear();
                block_sizes.clear();
                       free.clear();
            }
        }

        void release() noexcept {

            block_width = initial_block_width;

            decltype(blocks) temp{};
            blocks      = std::move(temp);
            block_sizes = {};
            free        = {};
        }
    };

} //chdr

#endif //CHDR_POLYTONIC_POOL_HPP