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

#include <cstddef>
#include <functional>
#include <stdexcept> // NOLINT(*-include-cleaner)
#include <vector>

namespace chdr {

    struct binary    {};   /** @brief Node with two children.   */
    struct tertiary  {};   /** @brief Node with three children. */
    struct quaternary{};   /** @brief Node with four children.  */
    struct quinary   {};   /** @brief Node with five children.  */
    struct senary    {};   /** @brief Node with six children.   */
    struct septenary {};   /** @brief Node with seven children. */
    struct octonary  {};   /** @brief Node with eight children. */

    template<typename>
    struct dimension {};
    template<> struct dimension<binary>     { static constexpr size_t Kd {2U}; };
    template<> struct dimension<tertiary>   { static constexpr size_t Kd {3U}; };
    template<> struct dimension<quaternary> { static constexpr size_t Kd {4U}; };
    template<> struct dimension<quinary>    { static constexpr size_t Kd {5U}; };
    template<> struct dimension<senary>     { static constexpr size_t Kd {6U}; };
    template<> struct dimension<septenary>  { static constexpr size_t Kd {7U}; };
    template<> struct dimension<octonary>   { static constexpr size_t Kd {8U}; };

    template<typename T, typename compare = std::less<T>, const size_t Kd = dimension<binary>::Kd>
    class heap {

        static_assert(Kd >= 2, "Template parameter D must be greater than or equal to 2.");

    private:

        std::vector<T> m_data;
        compare m_compare;

        size_t index_of(const T& _item) const {
            return static_cast<size_t>(&(_item) - &(top()));
        }

        constexpr void sort_up(const T& _item) {

            if (size() > 1U) {

                auto i = index_of(_item);
                while (i > 1U) {

                    const auto p = i / Kd;

                    if (p > 0U && m_compare(m_data[p], m_data[i])) {
                        std::swap(m_data[i], m_data[p]);
                        i = p;
                    }
                    else {
                        break;
                    }
                }
            }
        }

        constexpr void sort_down(const T& _item) {

            if (size() > 1U) {

                auto i = index_of(_item);
                while (i > 1U) {

                    const auto c0 =  i * Kd;
                    const auto cn = c0 + (Kd - 1U);

                    if (cn < m_data.size()) {

                        size_t min{};

                        if constexpr (Kd == 2U) {
                            min = (cn < m_data.size() && m_compare(m_data[c0], m_data[cn])) ? cn : c0;
                        }
                        else {

                            min = i;

                            IVDEP
                            VECTOR_ALWAYS
                            for (auto j = c0; j <= cn && j < m_data.size(); ++j) {
                                if (m_compare(m_data[min], m_data[j])) {
                                    min = j;
                                }
                            }
                        }

                        if (m_compare(m_data[i], m_data[min])) {
                            std::swap(m_data[i], m_data[min]);
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

        heap(const size_t& _capacity = 0U) : m_data() {
            m_data.reserve(_capacity + 1U);
            m_data.push_back({}); // Add super element.
        }

        [[nodiscard]] constexpr bool    empty() const { return size() == 0U;       }
        [[nodiscard]] constexpr size_t   size() const { return m_data.size() - 1U; }

        [[nodiscard]] constexpr const T& front() { return top(); }

        [[nodiscard]] constexpr const T& top() const {

#ifndef NDEBUG
            if (empty()) {
                throw std::underflow_error("Heap is empty");
            }
#endif

            if constexpr (std::is_pointer_v<T>) {
                return static_cast<T>(begin().base());
            }
            else {
                return *begin();
            }
        }

        [[nodiscard]] constexpr const T& back() const {

#ifndef NDEBUG
            if (empty()) {
                throw std::underflow_error("Heap is empty");
            }
#endif

            if constexpr (std::is_pointer_v<T>) {
                return static_cast<T>(end().base());
            }
            else {
                return *end();
            }
        }

        [[maybe_unused]] constexpr void enqueue(const T& _item) { push(_item); }

        [[maybe_unused]] constexpr void enqueue(T&& _item) { emplace(_item); }
        
        [[maybe_unused]] constexpr void push(const T& _item) {
            m_data.push_back(_item);
            sort_up(m_data.back());
        }

        [[maybe_unused]] constexpr void push(T&& _item) {
            m_data.push_back(std::move(_item));
            sort_up(m_data.back());
        }

        [[maybe_unused]] constexpr void emplace(T&& _item) {
            m_data.emplace_back(std::move(_item));
            sort_up(m_data.back());
        }

        [[maybe_unused]] constexpr void remove(const T& _item) {

#ifndef NDEBUG
            if (empty()) {
                throw std::underflow_error("Heap is empty");
            }
#endif

            auto i = index_of(_item);
            if (i < size()) {

                if (i == size() - 1U) {
                    m_data.pop_back(); // If the item to remove is the last item, just remove it.
                }
                else {
                    m_data[i] = std::move(m_data.back());
                    m_data.pop_back();

                    if (size() > 1U) {

                        // Restore heap property:
                        if (i > 1U && m_compare(m_data[i], m_data[i / Kd])) {
                            sort_up(m_data[i]);
                        }
                        else {
                            sort_down(m_data[i]);
                        }
                    }
                }
            }
            else {

    #ifndef NDEBUG
                throw std::runtime_error("Heap::remove(const T& _item): (Out of Bounds) Item does not exist in Heap.");
    #endif
            }
        }

        [[maybe_unused]] constexpr T dequeue() {

            T result;

            if (!empty()) {

                result(std::move(top()));

                if (size() > 0U) {
                    m_data[1U] = std::move(m_data.back());
                }
                m_data.pop_back();
            }
#ifndef NDEBUG
            else {
                throw std::underflow_error("Heap is empty");
            }
#endif //!NDEBUG

            sort_down(m_data[1U]);

            return result;
        }

        [[maybe_unused]] constexpr void pop() {

            if (!empty()) {
                if (size() > 0U) {
                    m_data[1U] = std::move(m_data.back());
                }
                m_data.pop_back();
            }
#ifndef NDEBUG
            else {
                throw std::underflow_error("Heap is empty");
            }
#endif //!NDEBUG

            sort_down(m_data[1U]);
        }

        [[maybe_unused]] constexpr void pop_back() {

            if (!empty()) {
                m_data.pop_back();
            }
#ifndef NDEBUG
            else {
                throw std::underflow_error("Heap is empty");
            }
#endif //!NDEBUG
        }

        [[maybe_unused]] constexpr void reheapify(const T& _item) {
            sort_up(m_data[index_of(_item)]);
        }

        [[maybe_unused]] constexpr bool contains(T& _item) {

            constexpr bool result = !empty();

            if (result) {
                constexpr const auto& i = index_of(_item);
                result = i < m_data.size() && _item == m_data[i];
            }

            return result;
        }

        [[maybe_unused]] constexpr void reserve(const size_t& _capacity) {
            m_data.reserve(_capacity);
        }

        [[maybe_unused]] constexpr void clear() {
            m_data.erase(begin(), end());
        }

        [[maybe_unused]] constexpr void shrink_to_fit() {
            m_data.shrink_to_fit();
        }

#ifndef HEAP_SUPPRESS_EXCEPTION_WARNING
        [[deprecated("This function does not perform bounds checking in 'runtime' compiled builds.\nSuppress this warning by defining \"HEAP_SUPPRESS_UNSAFE_WARNING\".")]]
#endif

        [[maybe_unused]] const T& operator[](const size_t& _index) const {

#ifndef NDEBUG
            return m_data.at(_index + 1U);
#else
            return m_data[_index + 1U];
#endif // NDEBUG
        }

        using               iterator_t = typename std::vector<T>::iterator;
        using         const_iterator_t = typename std::vector<T>::const_iterator;
        using       reverse_iterator_t = typename std::vector<T>::reverse_iterator;
        using const_reverse_iterator_t = typename std::vector<T>::const_reverse_iterator;

        [[maybe_unused]] [[nodiscard]]       iterator_t  begin()       { return m_data.begin()  + 1U; }
        [[maybe_unused]] [[nodiscard]] const_iterator_t  begin() const { return m_data.begin()  + 1U; }
        [[maybe_unused]] [[nodiscard]] const_iterator_t cbegin() const { return m_data.cbegin() + 1U; }

        [[maybe_unused]] [[nodiscard]]       iterator_t  end()       { return m_data.end();  }
        [[maybe_unused]] [[nodiscard]] const_iterator_t  end() const { return m_data.end();  }
        [[maybe_unused]] [[nodiscard]] const_iterator_t cend() const { return m_data.cend(); }

        [[maybe_unused]] [[nodiscard]]       reverse_iterator_t  rbegin()       { return m_data.rbegin();  }
        [[maybe_unused]] [[nodiscard]] const_reverse_iterator_t  rbegin() const { return m_data.rbegin();  }
        [[maybe_unused]] [[nodiscard]] const_reverse_iterator_t crbegin() const { return m_data.crbegin(); }

        [[maybe_unused]] [[nodiscard]]       reverse_iterator_t  rend()       { return m_data.rend();  }
        [[maybe_unused]] [[nodiscard]] const_reverse_iterator_t  rend() const { return m_data.rend();  }
        [[maybe_unused]] [[nodiscard]] const_reverse_iterator_t crend() const { return m_data.crend(); }

    };

} // chdr

#endif