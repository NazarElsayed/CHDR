#ifndef CIRCULAR_QUEUE_HPP
#define CIRCULAR_QUEUE_HPP

#include <cstddef>
#include <utility>
#include <vector>

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

    [[maybe_unused, nodiscard]] constexpr bool empty() const { return m_count == 0U; }

    [[maybe_unused, nodiscard]] constexpr bool full() const { return m_count == m_capacity; }

    [[maybe_unused, nodiscard]] constexpr const size_t& size() const { return m_count; }

    constexpr void push(const T& _value) {

        if (full()) {
            m_head = (m_head + 1U) % m_capacity;
        }
        else {
            ++m_count;
        }

        c[m_tail] = _value;
        m_tail    = (m_tail + 1U) % m_capacity;
    }

    constexpr void push(T&& _value) {

        if (full()) {
            m_head = (m_head + 1U) % m_capacity;
        }
        else {
            ++m_count;
        }

        c[m_tail] = _value;
        m_tail    = (m_tail + 1U) % m_capacity;
    }

    constexpr void emplace(T&& _value) {
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
    constexpr void emplace(Args&&... args) {
        if (full()) {
            m_head = (m_head + 1U) % m_capacity;
        }
        else {
            ++m_count;
        }

        c[m_tail] = T(std::forward<Args>(args)...);
        m_tail    = (m_tail + 1U) % m_capacity;
    }
    
    constexpr void enqueue(const T& _value) { push(_value); }

    constexpr void enqueue(T&& _value) { push(_value); }

    constexpr T dequeue() {

        if (empty()) {
            throw std::underflow_error("Container is empty");
        }

        T value(std::move(c[m_head]));
        m_head = (m_head + 1U) % m_capacity;
        --m_count;
        return value;
    }

    constexpr void pop() {

        if (empty()) {
            throw std::underflow_error("Container is empty");
        }

        m_head = (m_head + 1U) % m_capacity;
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

    constexpr void clear() {
        m_head  = 0U;
        m_tail  = 0U;
        m_count = 0U;
    }

    constexpr void reserve(const size_t& _new_capacity) {
        
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

    using       iterator_t = typename Container::iterator;
    using const_iterator_t = typename Container::const_iterator;

    constexpr iterator_t begin() { return c.begin() + m_head; }
    constexpr iterator_t   end() { return c.begin() + m_tail; }

    constexpr const_iterator_t begin() const { return c.begin() + m_head; }
    constexpr const_iterator_t   end() const { return c.begin() + m_tail; }
};

#endif //CIRCULAR_QUEUE_HPP