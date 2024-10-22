#ifndef CHDR_STABLEFORWARDBUF_HPP
#define CHDR_STABLEFORWARDBUF_HPP

#include <forward_list>
#include <vector>

namespace CHDR {

    template <typename T, size_t BlockWidth = 1024U>
    class StableForwardBuf {

    private:

        std::forward_list<std::vector<T>> m_Blocks;

    public:

        constexpr explicit StableForwardBuf() : m_Blocks() {
            Expand();
        }

        constexpr StableForwardBuf(const std::initializer_list<size_t>& _items) : m_Blocks() {

            Expand();

            for (const auto& item : _items) {
                Push(item);
            }
        }

        void Expand() {
            m_Blocks.emplace_front(std::vector<T>()).reserve(BlockWidth);
        }

        T& Push(const T& _item) {

            if (m_Blocks.front().size() >= BlockWidth) {
                Expand();
            }

            return m_Blocks.front().push_back(_item);
        }

        T& Push(T&& _item) {

            if (m_Blocks.front().size() >= BlockWidth) {
                Expand();
            }

            return m_Blocks.front().push_back(std::move(_item));
        }

        T& Emplace(T&& _item) {

            if (m_Blocks.front().size() >= BlockWidth) {
                Expand();
            }

            return m_Blocks.front().emplace_back(std::move(_item));
        }

        void Clear() {
            m_Blocks.clear();
        }

        template <bool Const>
        class StableIterator {

        private:

            using BlockIterator = typename std::conditional<
                    Const,
                    typename std::forward_list<std::vector<T>>::const_iterator,
                    typename std::forward_list<std::vector<T>>::iterator
                >::type;

            using ElementIterator = typename std::conditional<
                    Const,
                    typename std::vector<T>::const_iterator,
                    typename std::vector<T>::iterator
                >::type;

              BlockIterator   block_iter;
              BlockIterator   block_end;
            ElementIterator element_iter;

        public:

            using iterator_category = std::forward_iterator_tag;

            using      value_type = T;
            using difference_type = std::ptrdiff_t;

            using   pointer = typename std::conditional<Const, const T*, T*>::type;
            using reference = typename std::conditional<Const, const T&, T&>::type;

            StableIterator(BlockIterator b_iter, BlockIterator b_end) : block_iter(b_iter), block_end(b_end) {
                if (block_iter != block_end) {
                    element_iter = block_iter->begin();
                }
            }

            StableIterator& operator++() {

                ++element_iter;
                if (element_iter == block_iter->end()) {
                    ++block_iter;
                    if (block_iter != block_end) {
                        element_iter = block_iter->begin();
                    }
                }
                return *this;
            }

            StableIterator operator++(int) {
                auto tmp = *this;
                ++(*this);
                return tmp;
            }

            reference operator*() const { return   *element_iter;  }
            pointer  operator->() const { return &(*element_iter); }

            bool operator==(const StableIterator& other) const {
                return block_iter == other.block_iter && (block_iter == block_end || element_iter == other.element_iter);
            }

            bool operator!=(const StableIterator& other) const { return !(*this == other); }
        };

        using       iterator = StableIterator<false>;
        using const_iterator = StableIterator<true>;

              iterator begin()        { return       iterator(m_Blocks.begin(), m_Blocks.end()); }
        const_iterator begin()  const { return const_iterator(m_Blocks.begin(), m_Blocks.end()); }
        const_iterator cbegin() const { return const_iterator(m_Blocks.begin(), m_Blocks.end()); }

              iterator end()          { return       iterator(m_Blocks.end(), m_Blocks.end()); }
        const_iterator end()    const { return const_iterator(m_Blocks.end(), m_Blocks.end()); }
        const_iterator cend()   const { return const_iterator(m_Blocks.end(), m_Blocks.end()); }
    };

} // CHDR

#endif //CHDR_STABLEFORWARDBUF_HPP