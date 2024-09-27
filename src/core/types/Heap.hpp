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

    // TODO: Make heap use size_t!

    struct IHeapItem {
        int m_HeapIndex;
    };

    template<typename T, typename Comparator>
    class Heap {

        static_assert(std::is_base_of_v<IHeapItem, T>, "T must derive from IHeapItem");

    private:
        std::vector<T> m_Data;
        Comparator compare;

    public:

        [[nodiscard]] constexpr     bool Empty() const { return m_Data.empty(); }
        [[nodiscard]] constexpr   size_t  Size() const { return m_Data. size(); }
        [[nodiscard]] constexpr const T&   Top() const { return m_Data.front(); }
        [[nodiscard]] constexpr const T&  Back() const { return m_Data. back(); }

        constexpr void Add(const T& _item) {

            m_Data.push_back(_item);
            m_Data.back().m_HeapIndex = m_Data.size() - 1U;

            SortUp(m_Data.back());
        }

        constexpr void Add(const T&& _item) {

            m_Data.push_back(_item);
            m_Data.back().m_HeapIndex = m_Data.size() - 1U;

            SortUp(m_Data.back());
        }

        constexpr void Emplace(T&& _item) {

            m_Data.emplace_back(std::move(_item));
            m_Data.back().m_HeapIndex = m_Data.size() - 1U;

            SortUp(m_Data.back());
        }

        constexpr void Remove(const T& _item) {

            const auto index = static_cast<unsigned>(_item.m_HeapIndex);

            if (index < m_Data.size()) {

                if (index == m_Data.size() - 1U) {

                    // If the item to remove is the last item, just remove it.
                    m_Data.pop_back();
                }
                else {

                    // Swap with the last element and remove the last element.
                    Swap(m_Data[index], m_Data.back());
                    m_Data.pop_back();

                    // Restore the heap property.
                    if (index > 0U && compare(m_Data[index], m_Data[(index - 1U) / 2U])) {
                        SortUp(m_Data[index]);
                    }
                    else {
                        SortDown(m_Data[index]);
                    }
                }
            }
            else {
                // TODO: Throw out of bounds exception.
            }
        }

        constexpr void RemoveFirst() {

            Swap(m_Data[0], m_Data.back());
            m_Data.pop_back();

            SortDown(m_Data[0]);
        }

        constexpr void RemoveLast() {

            if (!m_Data.empty()) {
                m_Data.erase(m_Data.begin());

                if (!m_Data.empty()) {
                    SortUp(m_Data[0]);
                }
            }
        }

        constexpr void Update(T& _item) {
            SortUp(m_Data[_item.m_HeapIndex]);
        }

        constexpr void SortUp(T& _item) {

            int itemIndex   = _item.m_HeapIndex;
            int parentIndex = (itemIndex - 1) / 2;

            while (true) {

                if (!compare(m_Data[parentIndex], m_Data[itemIndex])) {
                    break;
                }

                Swap(m_Data[itemIndex], m_Data[parentIndex]);
                itemIndex   = (itemIndex - 1) / 2;
                parentIndex = (itemIndex - 1) / 2;
            }
        }

        constexpr void SortDown(T& _item) {

            int itemIndex = _item.m_HeapIndex;

            int leftChildIndex  = (itemIndex * 2) + 1;
            int rightChildIndex = leftChildIndex + 1;

            while (true) {

                if (leftChildIndex < static_cast<signed>(m_Data.size())) {

                    int swapIndex = leftChildIndex;

                    if (rightChildIndex < static_cast<signed>(m_Data.size())) {
                        if (compare(m_Data[leftChildIndex], m_Data[rightChildIndex])) {
                            swapIndex = rightChildIndex;
                        }
                    }

                    if (compare(m_Data[itemIndex], m_Data[swapIndex])) {
                        Swap(m_Data[itemIndex], m_Data[swapIndex]);

                        itemIndex = swapIndex;
                        leftChildIndex  = (itemIndex * 2) + 1;
                        rightChildIndex = leftChildIndex + 1;
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

        constexpr void Swap(T& _item1, T& _item2) {

            if (&_item1 != &_item2) {
                std::swap(_item1, _item2);
                std::swap(_item1.m_HeapIndex, _item2.m_HeapIndex);
            }
        }

        constexpr bool Contains(T& _item) {
            return _item == m_Data[_item.m_HeapIndex];
        }

        constexpr void Clear() {
            m_Data.clear();
        }

        constexpr void printHeap() {

            std::cout << std::endl << "Current Heap:" << "\n";

            const int size = m_Data.size();
            for (int i = 0; i < size; i++) {
                std::cout << m_Data[0].m_FScore << "\n";
                RemoveFirst();
            }

            std::cout << std::endl;
        }

#ifndef HEAP_SUPPRESS_EXCEPTION_WARNING
        [[deprecated("This function does not perform bounds checking in 'runtime' compiled builds.\nSuppress this warning by defining \"HEAP_SUPPRESS_UNSAFE_WARNING\".")]]
#endif
        const T& operator[](const size_t& _index) const {

#ifndef NDEBUG
            return m_Data.at(_index);
#else
            return m_Data[_index];
#endif // NDEBUG
        }

        using               iterator = typename std::vector<T>::iterator;
        using         const_iterator = typename std::vector<T>::const_iterator;
        using       reverse_iterator = typename std::vector<T>::reverse_iterator;
        using const_reverse_iterator = typename std::vector<T>::const_reverse_iterator;

              iterator  begin()       { return m_Data.begin();  }
        const_iterator  begin() const { return m_Data.begin();  }
        const_iterator cbegin() const { return m_Data.cbegin(); }

              iterator  end()       { return m_Data.end();  }
        const_iterator  end() const { return m_Data.end();  }
        const_iterator cend() const { return m_Data.cend(); }

              reverse_iterator  rbegin()       { return m_Data.rbegin();  }
        const_reverse_iterator  rbegin() const { return m_Data.rbegin();  }
        const_reverse_iterator crbegin() const { return m_Data.crbegin(); }

              reverse_iterator  rend()       { return m_Data.rend();  }
        const_reverse_iterator  rend() const { return m_Data.rend();  }
        const_reverse_iterator crend() const { return m_Data.crend(); }
    };
}

#endif