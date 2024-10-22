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

    struct IHeapItem {
        size_t m_HeapIndex;
    };

    template<typename T, size_t D = 2U, typename Comparator = std::less<T>>
    class Heap {

        static_assert(std::is_base_of_v<IHeapItem, std::remove_pointer_t<std::remove_cv_t<std::remove_reference_t<T>>>>, "T must derive from IHeapItem");

    private:

        std::vector<T> m_Data;
        Comparator m_Comparator;

        template<typename U>
        struct heap_index_accessor {
            static size_t& get(U& item) {
                return item.m_HeapIndex;
            }
        };

        template<typename U>
        struct heap_index_accessor<U*> {
            static size_t& get(U* item) {
                return item->m_HeapIndex;
            }
        };

        template<typename U>
        struct heap_index_accessor<std::shared_ptr<U>> {
            static size_t& get(std::shared_ptr<U>& item) {
                return item->m_HeapIndex;
            }
        };

        size_t& get_heap_index(T& item) {
            return heap_index_accessor<T>::get(item);
        }

    public:

        Heap(const size_t& _capacity = 0U) : m_Data() {
            m_Data.reserve(_capacity + 1U);
            m_Data.push_back({});
        }

        [[nodiscard]] constexpr bool    Empty() const { return Size() == 0U;       }
        [[nodiscard]] constexpr size_t   Size() const { return m_Data.size() - 1U; }
        [[nodiscard]] constexpr const T&  Top() const { return m_Data[1U];         }
        [[nodiscard]] constexpr const T& Back() const { return m_Data.back();      }

        constexpr void Add(const T& _item) {
            m_Data.push_back(_item);
            get_heap_index(m_Data.back()) = Size();
            SortUp(m_Data.back());
        }

        constexpr void Add(T&& _item) {
            m_Data.push_back(std::move(_item));
            get_heap_index(m_Data.back()) = Size();
            SortUp(m_Data.back());
        }

        constexpr void Emplace(T&& _item) {
            m_Data.emplace_back(std::move(_item));
            get_heap_index(m_Data.back()) = Size();
            SortUp(m_Data.back());
        }

        void Reserve(const size_t& capacity) {
            m_Data.reserve(capacity);
        }

        constexpr void Remove(const T& _item) {

            auto& heapIndex = get_heap_index(const_cast<T&>(_item));
            if (heapIndex < Size()) {

                if (heapIndex == Size() - 1U) {
                    m_Data.pop_back(); // If the item to remove is the last item, just remove it.
                }
                else {
                    m_Data[heapIndex] = std::move(m_Data.back());
                    m_Data.pop_back();

                    if (Size() > 1U) {

                        // Restore heap property:
                        if (heapIndex > 1U && m_Comparator(m_Data[heapIndex], m_Data[heapIndex / D])) {
                            SortUp(m_Data[heapIndex]);
                        }
                        else {
                            SortDown(m_Data[heapIndex]);
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
            SortUp(m_Data[get_heap_index(_item)]);
        }

        constexpr bool Contains(T& _item) {

            const auto& heapIndex = get_heap_index(_item);

            return !Empty() && heapIndex < m_Data.size() && _item == m_Data[heapIndex];
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

    private:

        constexpr void SortUp(T& _item) {

            if (Size() > 1U) {

                auto itemIndex = get_heap_index(_item);
                auto parentIndex = itemIndex / D;

                while (itemIndex > 1U) {

                    if (m_Comparator(m_Data[parentIndex], m_Data[itemIndex])) {
                        std::swap(m_Data[itemIndex], m_Data[parentIndex]);
                        itemIndex = parentIndex;
                        parentIndex = itemIndex / D;
                    }
                    else {
                        break;
                    }
                }
            }
        }

        constexpr void SortDown(T& _item) {

            if (Size() > 1U) {

                auto itemIndex = get_heap_index(_item);

                auto leftChildIndex = itemIndex * D;
                auto rightChildIndex = leftChildIndex + 1U;

                while (leftChildIndex < m_Data.size()) {

                    auto swapIndex = leftChildIndex;
                    if (rightChildIndex < m_Data.size() && m_Comparator(m_Data[leftChildIndex], m_Data[rightChildIndex])) {
                        swapIndex = rightChildIndex;
                    }
                    if (m_Comparator(m_Data[itemIndex], m_Data[swapIndex])) {
                        std::swap(m_Data[itemIndex], m_Data[swapIndex]);
                        itemIndex = swapIndex;

                        leftChildIndex = itemIndex * D;
                        rightChildIndex = leftChildIndex + 1U;
                    }
                    else {
                        break;
                    }
                }
            }
        }

    };
}

#endif