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

        static_assert(D >= 2, "Template parameter D must be greater than or equal to 2.");

    private:

        std::vector<T> m_Data;
        Compare m_Compare;

        size_t index_of(const T& _item) const {
            return static_cast<size_t>(&(_item) - &(Top()));
        }

        constexpr void SortUp(T& _item) {

            if (Size() > 1U) {

                auto i = index_of(_item);

                while (i > 1U) {

                    const auto p = i / D;

                    if (p > 0U && m_Compare(m_Data[p], m_Data[i])) {
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

                while (i > 1U) {

                    auto c0 = i * D;
                    auto cn = c0 + (D - 1U);

                    if (cn < m_Data.size()) {

                        size_t min{};

                        if constexpr (D == 2U) {
                            min = (cn < m_Data.size() && m_Compare(m_Data[c0], m_Data[cn])) ? cn : c0;
                        }
                        else {

                            min = i;
                            for (auto j = c0; j <= cn && j < m_Data.size(); ++j) {
                                if (m_Compare(m_Data[min], m_Data[j])) {
                                    min = j;
                                }
                            }
                        }

                        if (m_Compare(m_Data[i], m_Data[min])) {
                            std::swap(m_Data[i], m_Data[min]);
                            i = min;
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

        [[nodiscard]] constexpr bool    Empty() const { return Size() == 0U;              }
        [[nodiscard]] constexpr size_t   Size() const { return m_Data.size() - 1U;        }

        [[nodiscard]] constexpr const T& Top() const {

            if constexpr (std::is_pointer_v<T>) {
                return static_cast<T>(begin().base());
            }
            else {
                return *begin();
            }
        }

        [[nodiscard]] constexpr const T& Back() const {

            if constexpr (std::is_pointer_v<T>) {
                return static_cast<T>(end().base());
            }
            else {
                return *end();
            }
        }

        [[maybe_unused]] constexpr void Add(const T& _item) {
            m_Data.push_back(_item);
            SortUp(m_Data.back());
        }

        [[maybe_unused]] constexpr void Add(T&& _item) {
            m_Data.push_back(std::move(_item));
            SortUp(m_Data.back());
        }

        [[maybe_unused]] constexpr void Emplace(T&& _item) {
            m_Data.emplace_back(std::move(_item));
            SortUp(m_Data.back());
        }

        [[maybe_unused]] constexpr void Remove(const T& _item) {

            auto i = index_of(_item);
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

        [[maybe_unused]] constexpr T PopTop() {

            T result(std::move(Top()));

            if (!Empty()) {
                if (Size() > 0U) {
                    m_Data[1U] = std::move(m_Data.back());
                }
                m_Data.pop_back();
            }
            SortDown(m_Data[1U]);

            return result;
        }

        [[maybe_unused]] constexpr T PopBack() {

            T result(std::move(Back()));

            if (!Empty()) {
                m_Data.pop_back();
            }

            return result;
        }

        [[maybe_unused]] constexpr void Update(T& _item) {
            SortUp(m_Data[index_of(_item)]);
        }

        [[maybe_unused]] constexpr bool Contains(T& _item) {

            const auto& i = index_of(_item);
            return !Empty() && i < m_Data.size() && _item == m_Data[i];
        }

        [[maybe_unused]] constexpr void Reserve(const size_t& capacity) {
            m_Data.reserve(capacity);
        }

        [[maybe_unused]] constexpr void Clear() {
            m_Data.erase(begin(), end());
        }

        [[maybe_unused]] constexpr void Trim() {
            m_Data.shrink_to_fit();
        }

#ifndef HEAP_SUPPRESS_EXCEPTION_WARNING
        [[deprecated("This function does not perform bounds checking in 'runtime' compiled builds.\nSuppress this warning by defining \"HEAP_SUPPRESS_UNSAFE_WARNING\".")]]
#endif

        [[maybe_unused]] const T& operator[](const size_t& _index) const {

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

        [[maybe_unused]] [[nodiscard]]       iterator  begin()       { return m_Data.begin()  + 1U; }
        [[maybe_unused]] [[nodiscard]] const_iterator  begin() const { return m_Data.begin()  + 1U; }
        [[maybe_unused]] [[nodiscard]] const_iterator cbegin() const { return m_Data.cbegin() + 1U; }

        [[maybe_unused]] [[nodiscard]]       iterator  end()       { return m_Data.end();  }
        [[maybe_unused]] [[nodiscard]] const_iterator  end() const { return m_Data.end();  }
        [[maybe_unused]] [[nodiscard]] const_iterator cend() const { return m_Data.cend(); }

        [[maybe_unused]] [[nodiscard]]       reverse_iterator  rbegin()       { return m_Data.rbegin();  }
        [[maybe_unused]] [[nodiscard]] const_reverse_iterator  rbegin() const { return m_Data.rbegin();  }
        [[maybe_unused]] [[nodiscard]] const_reverse_iterator crbegin() const { return m_Data.crbegin(); }

        [[maybe_unused]] [[nodiscard]]       reverse_iterator  rend()       { return m_Data.rend();  }
        [[maybe_unused]] [[nodiscard]] const_reverse_iterator  rend() const { return m_Data.rend();  }
        [[maybe_unused]] [[nodiscard]] const_reverse_iterator crend() const { return m_Data.crend(); }

    };
}

#endif