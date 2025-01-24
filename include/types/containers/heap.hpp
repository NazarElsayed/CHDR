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
#include <memory_resource>
#include <stdexcept> // NOLINT(*-include-cleaner)
#include <vector>

namespace chdr {

    template <typename T, typename Compare = std::less<T>, typename Container = std::pmr::vector<T>, size_t Kd = 2U>
    class heap {

        static_assert(Kd >= 2U, "Template parameter D must be greater than or equal to 2.");

    protected:
        Container c;
        Compare   comp;

    private:

        [[nodiscard]] HOT constexpr size_t index_of(const T& _item) const noexcept {
            return static_cast<size_t>(&(_item) - &(c[1U]));
        }

    public:

        HOT constexpr void sort_up(const T& _item) noexcept {
            
            if (size() <= 1U) {
                return;
            }

            auto i = index_of(_item);

            assert(i < size() && "(Out of Bounds) Item does not exist in Heap.");

            while (i > 1U) {
                const auto p = i / Kd;

                if (comp(c[p], c[i])) {
                    std::swap(c[i], c[p]);
                    i = p;
                }
                else {
                    break;
                }
            }
        }

        HOT constexpr void sort_down(const T& _item) noexcept {

            if (size() > 1U) {
                auto i = index_of(_item);

                assert(i < size() && "(Out of Bounds) Item does not exist in Heap.");

                while (i > 1U) {
                    const auto c0 = i * Kd;
                    const auto cn = c0 + (Kd - 1U);

                    if (cn < c.size()) {
                        size_t min{};

                        if constexpr (Kd == 2U) {
                            min = (cn < c.size() && comp(c[c0], c[cn])) ? cn : c0;
                        }
                        else {
                            min = i;

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

        static constexpr auto dimension_v { Kd };

        heap(std::pmr::memory_resource* _resource = std::pmr::get_default_resource()) : c(_resource) {
            c.reserve(1U);
            c.emplace_back(); // Add uninitialised super element.
        }

        heap(size_t _capacity, std::pmr::memory_resource* _resource = std::pmr::get_default_resource()) : c(_resource) {
            c.reserve(utils::min(_capacity, std::numeric_limits<size_t>::max() - 1U) + 1U);
            c.emplace_back(); // Add uninitialised super element.
        }

        explicit heap(const Container& _container) : c(_container.get_allocator().resource()), comp() {

            c.emplace_back(); // Add uninitialised super element.
            c.insert(
                c.end(),
                _container.begin(),
                _container.end()
            );

            if (!empty()) {
                for (size_t i = size() / Kd; i >= 1U; --i) {
                    sort_down(c[i]);
                }
            }
        }

        explicit heap(Container&& _container) : c(std::move(_container)), comp() {

            c.emplace_back(); // Add uninitialised super element.
            c.insert(
                c.end(),
                std::make_move_iterator(_container.begin()),
                std::make_move_iterator(_container.end())
            );

            if (!empty()) {
                for (size_t i = size() / Kd; i >= 1U; --i) {
                    sort_down(c[i]);
                }
            }
        }

        [[maybe_unused, nodiscard]] HOT constexpr bool empty() const noexcept { return size() == 0U; }

        [[maybe_unused, nodiscard]] constexpr size_t size() const noexcept { return c.size() - 1U; }

        [[maybe_unused, nodiscard]] constexpr size_t capacity() const noexcept { return c.capacity(); }

        [[maybe_unused, nodiscard]] HOT constexpr       T& front()       noexcept { return top(); }
        [[maybe_unused, nodiscard]] HOT constexpr const T& front() const noexcept { return top(); }

        [[maybe_unused, nodiscard]] HOT constexpr T& top() noexcept {

            assert(!empty() && "Heap is empty");

            if constexpr (std::is_pointer_v<T>) {
                return reinterpret_cast<T&>(begin().base());
            }
            else {
                return *begin();
            }
        }

        [[maybe_unused, nodiscard]] HOT constexpr const T& top() const noexcept {

            assert(!empty() && "Heap is empty");

            if constexpr (std::is_pointer_v<T>) {
                return reinterpret_cast<T&>(begin().base());
            }
            else {
                return *begin();
            }
        }

        [[maybe_unused, nodiscard]] HOT constexpr const T& back() const noexcept {

            assert(!empty() && "Heap is empty");

            if constexpr (std::is_pointer_v<T>) {
                return reinterpret_cast<T&>(end().base());
            }
            else {
                return *end();
            }
        }

        [[maybe_unused, nodiscard]] HOT constexpr T& back() noexcept {

            assert(!empty() && "Heap is empty");

            if constexpr (std::is_pointer_v<T>) {
                return reinterpret_cast<T&>(end().base());
            }
            else {
                return *end();
            }
        }

        [[maybe_unused]] HOT constexpr void enqueue(const T& _item) { push(_item); }

        template <class... Args>
        [[maybe_unused]] HOT constexpr void enqueue(Args&&... _args) { emplace(std::forward<Args>(_args)...); }

        [[maybe_unused]] HOT constexpr void push(const T& _item) {
            c.push_back(_item);
            sort_up(c.back());
        }

        [[maybe_unused]] HOT constexpr void push(T&& _item) {
            c.push_back(std::move(_item));
            sort_up(c.back());
        }

        template <class... Args>
        [[maybe_unused]] HOT constexpr void emplace(Args&&... _args) {
            c.emplace_back(std::forward<Args>(_args)...);
            sort_up(c.back());
        }

        [[maybe_unused]] HOT constexpr void enqueue_nosort(const T& _item) { push_nosort(_item); }

        template <class... Args>
        [[maybe_unused]] HOT constexpr void enqueue_nosort(Args&&... _args) { emplace_nosort(std::forward<Args>(_args)...); }

        [[maybe_unused]] HOT constexpr void push_nosort(const T& _item) { c.push_back(_item); }

        [[maybe_unused]] HOT constexpr void push_nosort(T&& _item) { c.push_back(std::move(_item)); }

        template <class... Args>
        [[maybe_unused]] HOT constexpr void emplace_nosort(Args&&... _args) { c.emplace_back(std::forward<Args>(_args)...); }

        [[maybe_unused]] constexpr void erase(const T& _item) noexcept {

            assert(!empty() && "Heap is empty");

            auto i = index_of(_item);

            assert(i < size() && "(Out of Bounds) Item does not exist in Heap.");

            if (i < size()) {

                if (i == size() - 1U) {
                    c.pop_back(); // If the item to remove is the last item, just remove it.
                }
                else {
                    c[i] = std::move(c.back());
                    c.pop_back();

                    // Restore heap property:
                    if (size() > 1U) {

                        if (i > 1U && comp(c[i], c[i / Kd])) {
                            sort_up(c[i]);
                        }
                        else {
                            sort_down(c[i]);
                        }
                    }
                }
            }
        }

        [[maybe_unused, nodiscard]] constexpr T dequeue() noexcept {
            assert(!empty() && "Heap is empty");

            T result;

            if (!empty()) {
                result(std::move(top()));

                if (size() > 0U) {
                    c[1U] = std::move(c.back());
                }
                c.pop_back();
            }

            sort_down(c[1U]);

            return result;
        }

        [[maybe_unused]] HOT constexpr void pop() noexcept {

            assert(!empty() && "Heap is empty");

            if (!empty()) {
                if (size() > 0U) {
                    c[1U] = std::move(c.back());
                }
                c.pop_back();
            }

            sort_down(c[1U]);
        }

        [[maybe_unused]] HOT constexpr void pop_back() noexcept {

            assert(!empty() && "Heap is empty");

            if (!empty()) {
                c.pop_back();
            }
        }

        [[maybe_unused, nodiscard]] constexpr bool contains(T& _item) noexcept {

            bool result = !empty();

            if (result) {
                const auto& i = index_of(_item);

                assert(i < size() && "(Out of Bounds) Item does not exist in Heap.");

                result = i < c.size() && _item == c[i];
            }

            return result;
        }

        [[maybe_unused]] constexpr void reserve(size_t _capacity) {
            c.reserve(_capacity);
        }

        [[maybe_unused]] constexpr void swap(heap& _other) noexcept {
            if (this != &_other) {
                std::swap(c,    _other.c);
                std::swap(comp, _other.comp);
            }
        }

        [[maybe_unused]] constexpr void clear() noexcept { c.clear(); }

        [[maybe_unused]] constexpr void shrink_to_fit() noexcept { c.shrink_to_fit(); }

        [[maybe_unused, nodiscard]] constexpr const T& at(size_t _index) const {
            return c.at(_index + 1U);
        }

        /**
         * @brief Wipes the heap, leaving it in an invalid state.
         *
         * @details This function completely wipes the internal container of the heap,
         *          leaving it free of all elements, including the super element.
         *
         * @warning Running this code leaves the heap in an invalid state.
         *          Valid use cases for this function are rare and, as such, its use is generally discouraged.
         *
         * @remarks Do not use this function unless you know what you are doing.
         */
        [[maybe_unused]]
#if __cplusplus >= 202002L
        constexpr
#endif
        void wipe() noexcept {
            try {
                c = std::move(decltype(c){});
            }
            catch (...) {
                c.clear();
                c.shrink_to_fit();
            }
            c.clear();
            c.shrink_to_fit();
        }

#ifndef HEAP_SUPPRESS_EXCEPTION_WARNING
        [[deprecated("This function does not perform bounds checking.\nSuppress this warning by defining \"HEAP_SUPPRESS_UNSAFE_WARNING\".")]]
#endif
        [[maybe_unused, nodiscard]] constexpr const T& operator[](size_t _index) const noexcept {
            return c[_index + 1U];
        }

        using iterator_t               = typename Container::              iterator;
        using const_iterator_t         = typename Container::        const_iterator;
        using reverse_iterator_t       = typename Container::      reverse_iterator;
        using const_reverse_iterator_t = typename Container::const_reverse_iterator;

        [[maybe_unused, nodiscard]] constexpr iterator_t        begin()       noexcept { return c.begin() + 1U; }
        [[maybe_unused, nodiscard]] constexpr const_iterator_t  begin() const noexcept { return c.begin() + 1U; }
        [[maybe_unused, nodiscard]] constexpr const_iterator_t cbegin() const noexcept { return c.cbegin() + 1U; }

        [[maybe_unused, nodiscard]] constexpr iterator_t        end()       noexcept { return c.end(); }
        [[maybe_unused, nodiscard]] constexpr const_iterator_t  end() const noexcept { return c.end(); }
        [[maybe_unused, nodiscard]] constexpr const_iterator_t cend() const noexcept { return c.cend(); }

        [[maybe_unused, nodiscard]] constexpr reverse_iterator_t        rbegin()       noexcept { return c.rbegin(); }
        [[maybe_unused, nodiscard]] constexpr const_reverse_iterator_t  rbegin() const noexcept { return c.rbegin(); }
        [[maybe_unused, nodiscard]] constexpr const_reverse_iterator_t crbegin() const noexcept { return c.crbegin(); }

        [[maybe_unused, nodiscard]] constexpr reverse_iterator_t        rend()       noexcept { return c.rend(); }
        [[maybe_unused, nodiscard]] constexpr const_reverse_iterator_t  rend() const noexcept { return c.rend(); }
        [[maybe_unused, nodiscard]] constexpr const_reverse_iterator_t crend() const noexcept { return c.crend(); }
    };
} //chdr

#endif