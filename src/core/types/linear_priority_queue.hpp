#ifndef LINEAR_PRIORITY_QUEUE_HPP
#define LINEAR_PRIORITY_QUEUE_HPP

#include <cstddef>
#include <functional>
#include <utility>
#include <vector>

#define LINEAR_PRIORITY_QUEUE_SUPPRESS_EXCEPTION_WARNING // Uncomment if you wish to remove the warning about possible unhandled exceptions.

template <typename T, typename Compare = std::less<T>, typename Container = std::vector<T>>
class linear_priority_queue {

protected:

    Container c;
    Compare   comp;

    [[nodiscard]] constexpr auto min_element() const { return std::min_element(c.begin(), c.end(), comp); }
    [[nodiscard]] constexpr auto max_element() const { return std::max_element(c.begin(), c.end(), comp); }

public:

    [[maybe_unused]]  linear_priority_queue() : c(Container()), comp(Compare()) {}
    [[maybe_unused]] ~linear_priority_queue() = default;

    [[maybe_unused]] explicit linear_priority_queue(const Compare& _comp) : comp(_comp) {}

    [[maybe_unused]] linear_priority_queue(const Compare& _comp, const Container& _c) : c(_c), comp(_comp) {}

    template <class InputIt>
    [[maybe_unused]] linear_priority_queue(InputIt _first, InputIt _last, const Compare& _comp = Compare()) :
        c(_first, _last),
        comp(_comp) {}

    [[maybe_unused]] linear_priority_queue (const linear_priority_queue& ) = default;
    [[maybe_unused]] linear_priority_queue (      linear_priority_queue&&) = default;

    [[maybe_unused]] linear_priority_queue& operator =(const linear_priority_queue& ) = default;
    [[maybe_unused]] linear_priority_queue& operator =(      linear_priority_queue&&) = default;

    [[nodiscard]] constexpr const T& front() const { return top(); }

    [[maybe_unused, nodiscard]] constexpr const T& top() const {

        if constexpr (std::is_pointer_v<T>) {
            return static_cast<T>(min_element().base());
        }
        else {
            return *min_element();
        }
    }

    [[maybe_unused, nodiscard]] constexpr const T& back() const {

        if constexpr (std::is_pointer_v<T>) {
            return static_cast<T>(max_element().base());
        }
        else {
            return *max_element();
        }
    }

    [[maybe_unused]] constexpr void push(const T& _value) { c.push_back(_value); }

    [[maybe_unused]] constexpr void push(T&& _value) { c.push_back(std::move(_value)); }

    template <class... Args>
    [[maybe_unused]] constexpr void emplace(Args&&... _args) { c.emplace_back(std::forward<Args>(_args)...); }

    [[maybe_unused]] constexpr void enqueue(const T& _item) { push(_item); }

    template <class... Args>
    [[maybe_unused]] constexpr void enqueue(Args&&... _args) { emplace(std::forward<Args>(_args)...); }

    [[maybe_unused]] constexpr T dequeue() {

        T result;

        if (!empty()) {

            const auto itr = min_element();

            if constexpr (std::is_pointer_v<T>) {
                result(std::move(static_cast<T>(itr.base())));
            }
            else {
                result(std::move(*itr));
            }

            c.erase(itr);
        }
        else {
            throw std::underflow_error("Container is empty");
        }

        return result;
    }

    [[maybe_unused]] constexpr void pop() {

        if (!empty()) {
            c.erase(min_element());
        }
#ifndef NDEBUG
        else {
            throw std::underflow_error("Container is empty");
        }
#endif //!NDEBUG

    }

    [[maybe_unused]] constexpr void pop_back() {

        if (!empty()) {
            c.erase(max_element());
        }
#ifndef NDEBUG
        else {
            throw std::underflow_error("Container is empty");
        }
#endif //!NDEBUG

    }

    [[maybe_unused]] constexpr void reserve(const size_t& _capacity) { c.reserve(_capacity); }

    [[maybe_unused]] constexpr void clear() { c.clear(); }

    [[maybe_unused]] constexpr void shrink_to_fit() { c.shrink_to_fit(); }

    [[maybe_unused]] constexpr void swap(linear_priority_queue& _other) noexcept {
        if (this != &_other) {
            std::swap(c, _other.c);
            std::swap(comp, _other.comp);
        }
    }

    [[maybe_unused, nodiscard]] constexpr bool empty() const { return c.empty(); }

    [[maybe_unused, nodiscard]] constexpr size_t size() const { return c.size(); }

    [[maybe_unused, nodiscard]] constexpr const T& at(const size_t& _index) const { return c.at(_index); }

#ifndef LINEAR_PRIORITY_QUEUE_SUPPRESS_EXCEPTION_WARNING
    [[deprecated("This function does not perform bounds checking.\nSuppress this warning by defining \"LINEAR_PRIORITY_QUEUE_SUPPRESS_EXCEPTION_WARNING\".")]]
#endif
    [[maybe_unused, nodiscard]] constexpr const T& operator[] (const size_t& _index) const { return c[_index]; }

    using               iterator_t = typename std::vector<T>::iterator;
    using         const_iterator_t = typename std::vector<T>::const_iterator;
    using       reverse_iterator_t = typename std::vector<T>::reverse_iterator;
    using const_reverse_iterator_t = typename std::vector<T>::const_reverse_iterator;

    [[maybe_unused, nodiscard]] constexpr       iterator_t  begin()       { return c.begin();  }
    [[maybe_unused, nodiscard]] constexpr const_iterator_t  begin() const { return c.begin();  }
    [[maybe_unused, nodiscard]] constexpr const_iterator_t cbegin() const { return c.cbegin(); }

    [[maybe_unused, nodiscard]] constexpr       iterator_t  end()       { return c.end();  }
    [[maybe_unused, nodiscard]] constexpr const_iterator_t  end() const { return c.end();  }
    [[maybe_unused, nodiscard]] constexpr const_iterator_t cend() const { return c.cend(); }

    [[maybe_unused, nodiscard]] constexpr       reverse_iterator_t  rbegin()       { return c.rbegin();  }
    [[maybe_unused, nodiscard]] constexpr const_reverse_iterator_t  rbegin() const { return c.rbegin();  }
    [[maybe_unused, nodiscard]] constexpr const_reverse_iterator_t crbegin() const { return c.crbegin(); }

    [[maybe_unused, nodiscard]] constexpr       reverse_iterator_t  rend()       { return c.rend();  }
    [[maybe_unused, nodiscard]] constexpr const_reverse_iterator_t  rend() const { return c.rend();  }
    [[maybe_unused, nodiscard]] constexpr const_reverse_iterator_t crend() const { return c.crend(); }

};

#endif //LINEAR_PRIORITY_QUEUE_HPP