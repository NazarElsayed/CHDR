/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_HEAP_HPP
#define CHDR_HEAP_HPP

#define HEAP_SUPPRESS_EXCEPTION_WARNING // Uncomment if you wish to remove the warning about possible unhandled exceptions.

namespace CHDR {

    template<typename T, size_t D = 2U, typename Compare = std::less<T>>
    class Heap {

    private:

        std::vector<T> m_Data;
        Compare m_Compare;

        size_t index_of(const T& _item) const {
            return static_cast<size_t>(&_item - &Top());
        }

        constexpr void SortUp(T& _item) {

            if (Size() > 1U) {

                auto i = index_of(_item);

                while (i > 1U) {

                    const auto p = i / D;

                    if (m_Compare(m_Data[p], m_Data[i])) {
                        std::swap(m_Data[i], m_Data[p]);
                        i = p;
                    }
                    else {
                        break;
                    }
                }
            }
        }

        constexpr void SortDown(T& _item) {

            if (Size() > 1U) {

                auto i = index_of(_item);

                while (true) {

                    auto l = i * D;
                    auto r = l + 1U;

                    if (l < m_Data.size()) {

                        auto s = (r < m_Data.size() && m_Compare(m_Data[l], m_Data[r])) ? r : l;

                        if (m_Compare(m_Data[i], m_Data[s])) {
                            std::swap(m_Data[i], m_Data[s]);
                            i = s;
                        }
                        else {
                            break;
                        }
                    }
                    else {
                        break;
                    }
                }
            }
        }

    public:

        Heap(const size_t& _capacity = 0U) : m_Data() {
            m_Data.reserve(_capacity + 1U);
            m_Data.push_back({}); // Add super element.
        }

        [[nodiscard]] constexpr bool    Empty() const { return Size() == 0U;       }
        [[nodiscard]] constexpr size_t   Size() const { return m_Data.size() - 1U; }
        [[nodiscard]] constexpr const T&  Top() const { return m_Data[1U];         }
        [[nodiscard]] constexpr const T& Back() const { return m_Data.back();      }

        constexpr void Add(const T& _item) {
            m_Data.push_back(_item);
            SortUp(m_Data.back());
        }

        constexpr void Add(T&& _item) {
            m_Data.push_back(std::move(_item));
            SortUp(m_Data.back());
        }

        constexpr void Emplace(T&& _item) {
            m_Data.emplace_back(std::move(_item));
            SortUp(m_Data.back());
        }

        constexpr void Remove(const T& _item) {

            auto& i = index_of(const_cast<T &>(_item));
            if (i < Size()) {

                if (i == Size() - 1U) {
                    m_Data.pop_back(); // If the item to remove is the last item, just remove it.
                }
                else {
                    m_Data[i] = std::move(m_Data.back());
                    m_Data.pop_back();

                    if (Size() > 1U) {

                        // Restore heap property:
                        if (i > 1U && m_Compare(m_Data[i], m_Data[i / D])) {
                            SortUp(m_Data[i]);
                        }
                        else {
                            SortDown(m_Data[i]);
                        }
                    }
                }
            }
            else {

    #ifndef NDEBUG
                throw std::runtime_error("Heap::Remove(const T& _item): (Out of Bounds) Item does not exist in Heap.");
    #endif
            }
        }

        constexpr void RemoveFirst() {
            m_Data[1U] = std::move(m_Data.back());
            m_Data.pop_back();
            SortDown(m_Data[1U]);
        }

        constexpr void RemoveLast() {
            if (!m_Data.empty()) {
                m_Data.pop_back();
            }
        }

        constexpr void Update(T& _item) {
            SortUp(m_Data[index_of(_item)]);
        }

        constexpr bool Contains(T& _item) {

            const auto& i = index_of(_item);
            return !Empty() && i < m_Data.size() && _item == m_Data[i];
        }

        constexpr void Reserve(const size_t& capacity) {
            m_Data.reserve(capacity);
        }

        constexpr void Clear() {
            m_Data.erase(begin(), end());
        }

        constexpr void Trim() {
            m_Data.shrink_to_fit();
        }

#ifndef HEAP_SUPPRESS_EXCEPTION_WARNING
        [[deprecated("This function does not perform bounds checking in 'runtime' compiled builds.\nSuppress this warning by defining \"HEAP_SUPPRESS_UNSAFE_WARNING\".")]]
#endif
        const T& operator[](const size_t& _index) const {

#ifndef NDEBUG
            return m_Data.at(_index + 1U);
#else
            return m_Data[_index + 1U];
#endif // NDEBUG
        }

        using               iterator = typename std::vector<T>::iterator;
        using         const_iterator = typename std::vector<T>::const_iterator;
        using       reverse_iterator = typename std::vector<T>::reverse_iterator;
        using const_reverse_iterator = typename std::vector<T>::const_reverse_iterator;

        [[nodiscard]]       iterator  begin()       { return m_Data.begin()  + 1U; }
        [[nodiscard]] const_iterator  begin() const { return m_Data.begin()  + 1U; }
        [[nodiscard]] const_iterator cbegin() const { return m_Data.cbegin() + 1U; }

        [[nodiscard]]       iterator  end()       { return m_Data.end();  }
        [[nodiscard]] const_iterator  end() const { return m_Data.end();  }
        [[nodiscard]] const_iterator cend() const { return m_Data.cend(); }

        [[nodiscard]]       reverse_iterator  rbegin()       { return m_Data.rbegin();  }
        [[nodiscard]] const_reverse_iterator  rbegin() const { return m_Data.rbegin();  }
        [[nodiscard]] const_reverse_iterator crbegin() const { return m_Data.crbegin(); }

        [[nodiscard]]       reverse_iterator  rend()       { return m_Data.rend();  }
        [[nodiscard]] const_reverse_iterator  rend() const { return m_Data.rend();  }
        [[nodiscard]] const_reverse_iterator crend() const { return m_Data.crend(); }

    };
}

#endif