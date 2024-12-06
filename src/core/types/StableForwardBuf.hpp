/*
* Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_STABLEFORWARDBUF_HPP
#define CHDR_STABLEFORWARDBUF_HPP

#include <forward_list>
#include <initializer_list>
#include <iterator>
#include <memory>

namespace chdr {

    template <typename T, size_t BlockWidth = 1024U>
    class StableForwardBuf {

    private:

        using Block = std::unique_ptr<T[]>;

        size_t m_index;
        std::forward_list<Block> m_Blocks;

        void Expand() {
            m_Blocks.emplace_front(std::make_unique<T[]>(BlockWidth));
            m_index = 0U;
        }

    public:

        [[maybe_unused]] constexpr explicit StableForwardBuf() : m_index(0U), m_Blocks() {
            m_Blocks.emplace_front(std::make_unique<T[]>(BlockWidth));
        }

        [[maybe_unused]] constexpr StableForwardBuf(const std::initializer_list<size_t>& _items) : m_index(0U), m_Blocks() {

            m_Blocks.emplace_front(std::make_unique<T[]>(BlockWidth));

            for (const auto& item : _items) {
                Push(item);
            }
        }

        [[maybe_unused]] [[nodiscard]] T& Push(const T& _item) {

            if (m_index >= BlockWidth) {
                Expand();
            }

            return m_Blocks.front()[m_index++] = _item;
        }

        [[maybe_unused]] [[nodiscard]] T& Emplace(T&& _item) {

            if (m_index >= BlockWidth) {
                Expand();
            }

            return m_Blocks.front()[m_index++] = std::move(_item);
        }

        [[maybe_unused]] void Clear() {
            m_Blocks.clear();
        }

        template<bool Const>
        class StableIterator {

        private:

            using BlockIterator = typename std::conditional<Const,
                typename std::forward_list<Block>::const_iterator,
                typename std::forward_list<Block>::iterator
            >::type;

            using ElementIterator = typename std::conditional<Const, const T*, T*>::type;

            BlockIterator block_iter;
            BlockIterator block_end;

            ElementIterator element_iter;

            size_t remaining_elements;

        public:

            using iterator_category [[maybe_unused]] = std::forward_iterator_tag;

            using      value_type [[maybe_unused]] = T;
            using difference_type [[maybe_unused]] = std::ptrdiff_t;

            using   pointer = typename std::conditional<Const, const T*, T*>::type;
            using reference = typename std::conditional<Const, const T&, T&>::type;

            [[maybe_unused]] StableIterator(BlockIterator b_iter, BlockIterator b_end, size_t remaining_elem) :
                block_iter(b_iter),
                block_end(b_end),
                remaining_elements(remaining_elem)
            {
                if (block_iter != block_end) {
                    element_iter = block_iter->get();
                }
            }

            StableIterator &operator ++() {

                ++element_iter;
                --remaining_elements;

                if ((element_iter - block_iter->get()) == BlockWidth || !remaining_elements) {

                    ++block_iter;
                    if (block_iter != block_end) {
                        element_iter = block_iter->get();
                    }
                }

                return *this;
            }

            StableIterator operator ++(int) {
                auto tmp = *this;
                ++(*this);
                return tmp;
            }

            reference operator* () const { return *element_iter; }
              pointer operator->() const { return  element_iter; }

            bool operator == (const StableIterator &other) const {
                return block_iter == other.block_iter &&
                    (block_iter == block_end || (element_iter == other.element_iter && remaining_elements == other.remaining_elements));
            }

            bool operator != (const StableIterator &other) const { return !(*this == other); }
        };

        using       iterator = StableIterator<false>;
        using const_iterator = StableIterator<true>;

        [[maybe_unused]] [[nodiscard]]       iterator  begin()       { return       iterator(m_Blocks.begin(), m_Blocks.end(), m_index); }
        [[maybe_unused]] [[nodiscard]] const_iterator  begin() const { return const_iterator(m_Blocks.begin(), m_Blocks.end(), m_index); }
        [[maybe_unused]] [[nodiscard]] const_iterator cbegin() const { return const_iterator(m_Blocks.begin(), m_Blocks.end(), m_index); }

        [[maybe_unused]] [[nodiscard]]       iterator  end()       { return       iterator(m_Blocks.end(), m_Blocks.end(), 0U); }
        [[maybe_unused]] [[nodiscard]] const_iterator  end() const { return const_iterator(m_Blocks.end(), m_Blocks.end(), 0U); }
        [[maybe_unused]] [[nodiscard]] const_iterator cend() const { return const_iterator(m_Blocks.end(), m_Blocks.end(), 0U); }
    };

} // CHDR

#endif // CHDR_STABLEFORWARDBUF_HPP