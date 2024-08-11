#ifndef CHDR_HEAP_HPP
#define CHDR_HEAP_HPP

namespace CHDR {

    struct IHeapItem {
        int m_HeapIndex;
    };

    template<typename T, typename Comparitor>
    class Heap {

        static_assert(std::is_base_of_v<IHeapItem, T>, "T must derive from IHeapItem");

    private:
        std::vector<T> m_Data;
        Comparitor compare;

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

            int swapIndex = 0;

            while (true) {

                if (leftChildIndex < m_Data.size()) {

                    swapIndex = leftChildIndex;

                    if (rightChildIndex < m_Data.size()) {
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
    };
}

#endif