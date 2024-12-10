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

    template<typename T, typename Compare = std::less<T>, typename Container = std::vector<T>, const size_t Kd = dimension<binary>::Kd>
    class heap {

        static_assert(Kd >= 2, "Template parameter D must be greater than or equal to 2.");

    protected:
        Container c;
        Compare comp;

    private:

        [[nodiscard]] size_t index_of(const T& _item) const noexcept {
            return static_cast<size_t>(&(_item) - &(c[1U]));
        }

        constexpr void sort_up(const T& _item) {

            if (size() > 1U) {

                auto i = index_of(_item);
                while (i > 1U) {

                    const auto p = i / Kd;

                    if (p > 0U && comp(c[p], c[i])) {
                        std::swap(c[i], c[p]);
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

                    if (cn < c.size()) {

                        size_t min{};

                        if constexpr (Kd == 2U) {
                            min = (cn < c.size() && comp(c[c0], c[cn])) ? cn : c0;
                        }
                        else {

                            min = i;

                            IVDEP
                            VECTOR_ALWAYS
                            for (auto j = c0; j <= cn && j < c.size(); ++j) {
                                if (comp(c[min], c[j])) {
                                    min = j;
                                }
                            }
                        }

                        if (comp(c[i], c[min])) {
                            std::swap(c[i], c[min]);
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

        heap(const size_t& _capacity = 0U) : c() {
            c.reserve(_capacity + 1U);
            c.push_back({}); // Add super element.
        }

        [[maybe_unused, nodiscard]] constexpr bool empty() const noexcept { return size() == 0U;  }

        [[maybe_unused, nodiscard]] constexpr size_t size() const noexcept { return c.size() - 1U; }

        [[maybe_unused, nodiscard]] constexpr T& front() { return top(); }

        [[maybe_unused, nodiscard]] constexpr const T& front() const { return top(); }

        [[maybe_unused, nodiscard]] constexpr T& top() {

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

        [[maybe_unused, nodiscard]] constexpr const T& top() const {

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

        [[maybe_unused, nodiscard]] constexpr const T& back() const {

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

        [[maybe_unused, nodiscard]] constexpr T& back() {

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

        template <class... Args>
        [[maybe_unused]] constexpr void enqueue(Args&&... _args) { emplace(std::forward<Args>(_args)...); }

        [[maybe_unused]] constexpr void push(const T& _item) {
            c.push_back(_item);
            sort_up(c.back());
        }

        [[maybe_unused]] constexpr void push(T&& _item) {
            c.push_back(std::move(_item));
            sort_up(c.back());
        }

        template <class... Args>
        [[maybe_unused]] constexpr void emplace(Args&&... _args) {
            c.emplace_back(std::forward<Args>(_args)...);
            sort_up(c.back());
        }

        [[maybe_unused]] constexpr void erase(const T& _item) {

#ifndef NDEBUG
            if (empty()) {
                throw std::underflow_error("Heap is empty");
            }
#endif

            auto i = index_of(_item);
            if (i < size()) {

                if (i == size() - 1U) {
                    c.pop_back(); // If the item to remove is the last item, just remove it.
                }
                else {
                    c[i] = std::move(c.back());
                    c.pop_back();

                    if (size() > 1U) {

                        // Restore heap property:
                        if (i > 1U && comp(c[i], c[i / Kd])) {
                            sort_up(c[i]);
                        }
                        else {
                            sort_down(c[i]);
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

        [[maybe_unused, nodiscard]] constexpr T dequeue() {

            T result;

            if (!empty()) {

                result(std::move(top()));

                if (size() > 0U) {
                    c[1U] = std::move(c.back());
                }
                c.pop_back();
            }
            else {
                throw std::underflow_error("Heap is empty");
            }

            sort_down(c[1U]);

            return result;
        }

        [[maybe_unused]] constexpr void pop() {

            if (!empty()) {
                if (size() > 0U) {
                    c[1U] = std::move(c.back());
                }
                c.pop_back();
            }
#ifndef NDEBUG
            else {
                throw std::underflow_error("Heap is empty");
            }
#endif //!NDEBUG

            sort_down(c[1U]);
        }

        [[maybe_unused]] constexpr void pop_back() {

            if (!empty()) {
                c.pop_back();
            }
#ifndef NDEBUG
            else {
                throw std::underflow_error("Heap is empty");
            }
#endif //!NDEBUG
        }

        [[maybe_unused]] constexpr void reheapify(const T& _item) {
            sort_up(c[index_of(_item)]);
        }

        [[maybe_unused, nodiscard]] constexpr bool contains(T& _item) noexcept {

            constexpr bool result = !empty();

            if (result) {
                constexpr const auto& i = index_of(_item);
                result = i < c.size() && _item == c[i];
            }

            return result;
        }

        [[maybe_unused]] constexpr void reserve(const size_t& _capacity) {
            c.reserve(_capacity);
        }

        [[maybe_unused]] constexpr void swap(heap& _other) noexcept {
            if (this != &_other) {
                std::swap(c, _other.c);
                std::swap(comp, _other.comp);
            }
        }

        [[maybe_unused]] constexpr void clear() {
            c.erase(begin(), end());
        }

        [[maybe_unused]] constexpr void shrink_to_fit() {
            c.shrink_to_fit();
        }

        [[maybe_unused, nodiscard]] constexpr const T& at(const size_t& _index) const {
            return c.at(_index);
        }

#ifndef HEAP_SUPPRESS_EXCEPTION_WARNING
        [[deprecated("This function does not perform bounds checking.\nSuppress this warning by defining \"HEAP_SUPPRESS_UNSAFE_WARNING\".")]]
#endif
        [[maybe_unused, nodiscard]] constexpr const T& operator[](const size_t& _index) const noexcept {
            return c[_index];
        }

        using               iterator_t = typename std::vector<T>::iterator;
        using         const_iterator_t = typename std::vector<T>::const_iterator;
        using       reverse_iterator_t = typename std::vector<T>::reverse_iterator;
        using const_reverse_iterator_t = typename std::vector<T>::const_reverse_iterator;

        [[maybe_unused, nodiscard]] constexpr       iterator_t  begin()       noexcept { return c.begin()  + 1U; }
        [[maybe_unused, nodiscard]] constexpr const_iterator_t  begin() const noexcept { return c.begin()  + 1U; }
        [[maybe_unused, nodiscard]] constexpr const_iterator_t cbegin() const noexcept { return c.cbegin() + 1U; }

        [[maybe_unused, nodiscard]] constexpr       iterator_t  end()       noexcept { return c.end();  }
        [[maybe_unused, nodiscard]] constexpr const_iterator_t  end() const noexcept { return c.end();  }
        [[maybe_unused, nodiscard]] constexpr const_iterator_t cend() const noexcept { return c.cend(); }

        [[maybe_unused, nodiscard]] constexpr       reverse_iterator_t  rbegin()       noexcept { return c.rbegin();  }
        [[maybe_unused, nodiscard]] constexpr const_reverse_iterator_t  rbegin() const noexcept { return c.rbegin();  }
        [[maybe_unused, nodiscard]] constexpr const_reverse_iterator_t crbegin() const noexcept { return c.crbegin(); }

        [[maybe_unused, nodiscard]] constexpr       reverse_iterator_t  rend()       noexcept { return c.rend();  }
        [[maybe_unused, nodiscard]] constexpr const_reverse_iterator_t  rend() const noexcept { return c.rend();  }
        [[maybe_unused, nodiscard]] constexpr const_reverse_iterator_t crend() const noexcept { return c.crend(); }

    };

} //chdr

#endif