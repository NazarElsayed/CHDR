#ifndef STACK_HPP
#define STACK_HPP

#include <cstddef>
#include <stack>
#include <vector>

namespace chdr {

    template <typename T, typename Container = std::vector<T>>
    class stack {

    private:

        using stack_t = std::stack<T, Container>;

        stack_t c;

    public:

        constexpr stack() = default;

        constexpr stack(const size_t& _capacity) {

            Container sequence;
            sequence.reserve(_capacity);

            c = stack_t(std::move(sequence));
        }

        constexpr stack(const Container& _sequence) : c(_sequence) {}

        constexpr stack(Container&& _sequence) : c(std::move(_sequence)) {}

        [[maybe_unused, nodiscard]] constexpr bool empty() const { return c.empty(); }

        [[maybe_unused, nodiscard]] constexpr size_t size() const { return c.size(); }

        [[maybe_unused, nodiscard]] constexpr T& front() { return top(); }

        [[maybe_unused, nodiscard]] constexpr const T& front() const { return top(); }

        [[maybe_unused, nodiscard]] constexpr T& top() { return c.top(); }

        [[maybe_unused, nodiscard]] constexpr const T& top() const { return c.top(); }

        [[maybe_unused]] constexpr void push(const T& _value) { c.push(_value); }

        [[maybe_unused]] constexpr void push(T&& _value) { c.push(std::move(_value)); }

        template <typename... Args>
        [[maybe_unused]] constexpr void emplace(Args&&... args) {
            c.emplace(std::forward<Args>(args)...);
        }

        [[maybe_unused]] constexpr void pop() { c.pop(); }

        [[maybe_unused]] constexpr void clear() {
            stack_t empty;
            std::swap(c, empty);
        }

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

}//chdr

#endif //STACK_HPP