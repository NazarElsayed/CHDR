/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef STACK_HPP
#define STACK_HPP

#include <cstddef>
#include <memory_resource>
#include <vector>

#include "include/utils/intrinsics.hpp"

namespace chdr {

    template <typename T>
    class stack {

    private:

        using stack_t = std::pmr::vector<T>;

        stack_t c;

    public:

        constexpr stack([[maybe_unused]] std::pmr::memory_resource* _resource = std::pmr::get_default_resource()) : c(_resource) {}

        constexpr stack(size_t _capacity, [[maybe_unused]] std::pmr::memory_resource* _resource = std::pmr::get_default_resource()) : c(_resource) {
            c.reserve(_capacity);
        }

        constexpr stack(const stack_t& _sequence) : c(_sequence) {}

        constexpr stack(stack_t&& _sequence) : c(std::move(_sequence)) {}

        [[maybe_unused, nodiscard]] HOT constexpr bool empty() const noexcept { return c.empty(); }

        [[maybe_unused, nodiscard]] constexpr size_t size() const noexcept { return c.size(); }

        [[maybe_unused, nodiscard]] HOT constexpr       T& front()       noexcept { return top(); }
        [[maybe_unused, nodiscard]] HOT constexpr const T& front() const noexcept { return top(); }

        [[maybe_unused, nodiscard]] HOT constexpr       T& top()       noexcept { return c.back(); }
        [[maybe_unused, nodiscard]] HOT constexpr const T& top() const noexcept { return c.back(); }

        [[maybe_unused]] HOT constexpr void push(const T& _value) { c.push(_value); }

        [[maybe_unused]] HOT constexpr void push(T&& _value) { c.push(std::move(_value)); }

        template <typename... Args>
        [[maybe_unused]] HOT constexpr void emplace(Args&&... _args) {
            c.emplace_back(std::forward<Args>(_args)...);
        }

        [[maybe_unused]] HOT constexpr void pop() { c.pop_back(); }

        [[maybe_unused]] constexpr void clear() {
            stack_t empty;
            std::swap(c, empty);
        }

        using               iterator_t = typename std::vector<T>::              iterator;
        using         const_iterator_t = typename std::vector<T>::        const_iterator;
        using       reverse_iterator_t = typename std::vector<T>::      reverse_iterator;
        using const_reverse_iterator_t = typename std::vector<T>::const_reverse_iterator;

        [[maybe_unused, nodiscard]] constexpr       iterator_t  begin()       noexcept { return c.begin();  }
        [[maybe_unused, nodiscard]] constexpr const_iterator_t  begin() const noexcept { return c.begin();  }
        [[maybe_unused, nodiscard]] constexpr const_iterator_t cbegin() const noexcept { return c.cbegin(); }

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

#endif //STACK_HPP