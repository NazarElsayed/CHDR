/*
* Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_STABLEFORWARDBUF_HPP
#define CHDR_STABLEFORWARDBUF_HPP

#include <cstddef>
#include <forward_list>
#include <initializer_list>
#include <iterator>
#include <memory>
#include <type_traits>

namespace chdr {

    template <typename T, const size_t BlockWidth = 1024U>
    class stable_forward_buf {

    private:

        using block_t = std::unique_ptr<T[]>; // NOLINT(*-avoid-c-arrays)

        size_t m_index;
        std::forward_list<block_t> m_blocks;

        void expand() {
            m_blocks.emplace_front(std::make_unique<T[]>(BlockWidth)); // NOLINT(*-avoid-c-arrays)
            m_index = 0U;
        }

    public:

        [[maybe_unused]] constexpr explicit stable_forward_buf() : m_index(0U), m_blocks() {
            m_blocks.emplace_front(std::make_unique<T[]>(BlockWidth)); // NOLINT(*-avoid-c-arrays)
        }

        [[maybe_unused]] constexpr stable_forward_buf(const std::initializer_list<size_t>& _items) : m_index(0U), m_blocks() {

            m_blocks.emplace_front(std::make_unique<T[]>(BlockWidth)); // NOLINT(*-avoid-c-arrays)

            for (const auto& item : _items) {
                push(item);
            }
        }

        [[maybe_unused]] [[nodiscard]] T& push(const T& _item) {

            if (m_index >= BlockWidth) {
                expand();
            }

            return m_blocks.front()[m_index++] = _item;
        }

        [[maybe_unused]] [[nodiscard]] T& emplace(T&& _item) {

            if (m_index >= BlockWidth) {
                expand();
            }

            return m_blocks.front()[m_index++] = std::move(_item);
        }

        [[maybe_unused]] void clear() {
            m_blocks.clear();
        }

        template<bool Const>
        class stable_iterator {

        private:

            using block_iterator_t = std::conditional_t<Const,
                typename std::forward_list<block_t>::const_iterator,
                typename std::forward_list<block_t>::iterator
            >;

            using element_iterator_t = std::conditional_t<Const, const T*, T*>;

            block_iterator_t m_block_iter;
            block_iterator_t m_block_end;

            element_iterator_t m_element_iter;

            size_t m_remaining_elements;

        public:

            using iterator_category_t [[maybe_unused]] = std::forward_iterator_tag;

            using      value_type [[maybe_unused]] = T;
            using difference_type [[maybe_unused]] = std::ptrdiff_t;

            using   pointer_t = std::conditional_t<Const, const T*, T*>;
            using reference_t = std::conditional_t<Const, const T&, T&>;

            [[maybe_unused]] stable_iterator(block_iterator_t _bIter, block_iterator_t _bEnd, const size_t _remainingElem) :
                m_block_iter(_bIter),
                m_block_end(_bEnd),
                m_remaining_elements(_remainingElem)
            {
                if (m_block_iter != m_block_end) {
                    m_element_iter = m_block_iter->get();
                }
            }

            stable_iterator &operator ++() {

                ++m_element_iter;
                --m_remaining_elements;

                if ((m_element_iter - m_block_iter->get()) == BlockWidth || (m_remaining_elements == 0U)) {

                    ++m_block_iter;
                    if (m_block_iter != m_block_end) {
                        m_element_iter = m_block_iter->get();
                    }
                }

                return *this;
            }

            stable_iterator operator ++(int) {
                auto tmp = *this;
                ++(*this);
                return tmp;
            }

            reference_t operator* () const { return *m_element_iter; }
              pointer_t operator->() const { return  m_element_iter; }

            bool operator == (const stable_iterator &_other) const {
                return m_block_iter == _other.m_block_iter &&
                    (m_block_iter == m_block_end || (m_element_iter == _other.m_element_iter && m_remaining_elements == _other.m_remaining_elements));
            }

            bool operator != (const stable_iterator &_other) const { return !(*this == _other); } // NOLINT(*-simplify)
        };

        using       iterator_t = stable_iterator<false>;
        using const_iterator_t = stable_iterator<true>;

        [[maybe_unused]] [[nodiscard]]       iterator_t  begin()       { return       iterator(m_blocks.begin(), m_blocks.end(), m_index); }
        [[maybe_unused]] [[nodiscard]] const_iterator_t  begin() const { return const_iterator(m_blocks.begin(), m_blocks.end(), m_index); }
        [[maybe_unused]] [[nodiscard]] const_iterator_t cbegin() const { return const_iterator(m_blocks.begin(), m_blocks.end(), m_index); }

        [[maybe_unused]] [[nodiscard]]       iterator_t  end()       { return       iterator(m_blocks.end(), m_blocks.end(), 0U); }
        [[maybe_unused]] [[nodiscard]] const_iterator_t  end() const { return const_iterator(m_blocks.end(), m_blocks.end(), 0U); }
        [[maybe_unused]] [[nodiscard]] const_iterator_t cend() const { return const_iterator(m_blocks.end(), m_blocks.end(), 0U); }
    };

} // chdr

#endif // CHDR_STABLEFORWARDBUF_HPP