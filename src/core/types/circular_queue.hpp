/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CIRCULAR_QUEUE_HPP
#define CIRCULAR_QUEUE_HPP

#include <cstddef>
#include <utility>
#include <vector>

namespace chdr {

    template <typename T, typename Container = std::vector<T>>
    class circular_queue {

    protected:

        Container c;

        size_t m_head;
        size_t m_tail;
        size_t m_count;
        size_t m_capacity;

    public:

        constexpr explicit circular_queue(const size_t& _capacity) :
            c(_capacity),
            m_head (0U),
            m_tail (0U),
            m_count(0U),
            m_capacity(_capacity) {}

        ~circular_queue() = default;

        circular_queue(const circular_queue&  _other)          = default;
        circular_queue(      circular_queue&& _other) noexcept = default;

        circular_queue& operator =(const circular_queue&  _other)          = default;
        circular_queue& operator =(      circular_queue&& _other) noexcept = default;

        [[maybe_unused, nodiscard]] constexpr bool empty() const noexcept { return m_count == 0U; }

        [[maybe_unused, nodiscard]] constexpr bool full() const noexcept { return m_count == m_capacity; }

        [[maybe_unused, nodiscard]] constexpr const size_t& size() const noexcept { return m_count; }

        [[maybe_unused]] constexpr void push(const T& _value) noexcept {

            if (full()) {
                m_head = (m_head + 1U) % m_capacity;
            }
            else {
                ++m_count;
            }

            c[m_tail] = _value;
            m_tail    = (m_tail + 1U) % m_capacity;
        }

        [[maybe_unused]] constexpr void push(T&& _value) noexcept {

            if (full()) {
                m_head = (m_head + 1U) % m_capacity;
            }
            else {
                ++m_count;
            }

            c[m_tail] = std::move(_value);
            m_tail    = (m_tail + 1U) % m_capacity;
        }

        [[maybe_unused]] constexpr void emplace(T&& _value) noexcept {

            if (full()) {
                m_head = (m_head + 1U) % m_capacity;
            }
            else {
                ++m_count;
            }

            c[m_tail] = std::forward<T>(_value);
            m_tail    = (m_tail + 1U) % m_capacity;
        }

        template <typename... Args>
        [[maybe_unused]] constexpr void emplace(Args&&... args) noexcept {

            if (full()) {
                m_head = (m_head + 1U) % m_capacity;
            }
            else {
                ++m_count;
            }

            c[m_tail] = T(std::forward<Args>(args)...);
            m_tail    = (m_tail + 1U) % m_capacity;
        }

        [[maybe_unused]] constexpr void enqueue(const T& _value) noexcept { push(_value); }

        [[maybe_unused]] constexpr void enqueue(T&& _value) noexcept { push(_value); }

        [[maybe_unused]] constexpr T dequeue() {

            if (empty()) {
                throw std::underflow_error("Container is empty");
            }

            T value(std::move(c[m_head]));
            m_head = (m_head + 1U) % m_capacity;
            --m_count;
            return value;
        }

        [[maybe_unused]] constexpr void pop() {

#ifndef NDEBUG
            if (empty()) {
                throw std::underflow_error("Container is empty");
            }
#endif //!NDEBUG

            m_head = (m_head + 1U) % m_capacity;
            --m_count;
        }

        [[maybe_unused]] constexpr void pop_back() {

#ifndef NDEBUG
            if (empty()) {
                throw std::underflow_error("Container is empty");
            }
#endif //!NDEBUG

            m_tail = (m_tail + m_capacity - 1U) % m_capacity;
            --m_count;
        }

        [[maybe_unused, nodiscard]] constexpr const T& top() const { return front(); }

        [[maybe_unused, nodiscard]] constexpr const T& front() const {

            if (empty()) {
                throw std::underflow_error("Container is empty");
            }

            return c[m_head];
        }

        [[maybe_unused, nodiscard]] constexpr const T& back() const {

            if (empty()) {
                throw std::underflow_error("Container is empty");
            }

            return c[(m_tail + m_capacity - 1U) % m_capacity];
        }

        [[maybe_unused]] constexpr void swap(circular_queue& _other) noexcept {

            if (this != &_other) {

                std::swap(c,          _other.c         );
                std::swap(m_head,     _other.m_head    );
                std::swap(m_tail,     _other.m_tail    );
                std::swap(m_count,    _other.m_count   );
                std::swap(m_capacity, _other.m_capacity);
            }
        }

        [[maybe_unused]] constexpr void clear() noexcept {
            m_head  = 0U;
            m_tail  = 0U;
            m_count = 0U;
        }

        [[maybe_unused]] constexpr void reserve(const size_t& _new_capacity) {

            if (_new_capacity > m_capacity) {

                Container new_c(_new_capacity);
                for (size_t i = 0U; i < m_count; ++i) {
                    new_c[i] = std::move(c[(m_head + i) % m_capacity]);
                }

                c          = std::move(new_c);
                m_capacity = _new_capacity;
                m_head     = 0U;
                m_tail     = m_count;
            }
        }

        using               iterator_t = typename Container::iterator;
        using         const_iterator_t = typename Container::const_iterator;
        using       reverse_iterator_t = typename Container::reverse_iterator;
        using const_reverse_iterator_t = typename Container::const_reverse_iterator;

        [[maybe_unused]] constexpr       iterator_t  begin()       noexcept { return c.begin() + m_head; }
        [[maybe_unused]] constexpr const_iterator_t  begin() const noexcept { return c.begin() + m_head; }
        [[maybe_unused]] constexpr const_iterator_t cbegin() const noexcept { return c.begin() + m_head; }

        [[maybe_unused]] constexpr       iterator_t  end()       noexcept { return c.begin() + m_tail; }
        [[maybe_unused]] constexpr const_iterator_t  end() const noexcept { return c.begin() + m_tail; }
        [[maybe_unused]] constexpr const_iterator_t cend() const noexcept { return c.begin() + m_tail; }

        [[maybe_unused]] constexpr       reverse_iterator_t  rbegin()       noexcept { return       reverse_iterator_t(end()); }
        [[maybe_unused]] constexpr const_reverse_iterator_t  rbegin() const noexcept { return const_reverse_iterator_t(end()); }
        [[maybe_unused]] constexpr const_reverse_iterator_t crbegin() const noexcept { return const_reverse_iterator_t(end()); }

        [[maybe_unused]] constexpr       reverse_iterator_t  rend()       noexcept { return       reverse_iterator_t(begin()); }
        [[maybe_unused]] constexpr const_reverse_iterator_t  rend() const noexcept { return const_reverse_iterator_t(begin()); }
        [[maybe_unused]] constexpr const_reverse_iterator_t crend() const noexcept { return const_reverse_iterator_t(begin()); }
    };

} //chdr

#endif //CIRCULAR_QUEUE_HPP