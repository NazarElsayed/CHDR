/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef STACK_HPP
#define STACK_HPP

/**
 * @file stack.hpp
 */

#include <cstddef>
#include <memory_resource>
#include <vector>

#include "include/utils/intrinsics.hpp"

namespace chdr {

    /**
     * @nosubgrouping
     * @class stack
     * @brief Implementation of a First-In-Last-Out (FILO) container.
     *
     * @details Manages a collection of elements, maintaining a FILO ordering.
     *
     * @tparam T Type of elements stored in the stack.
     *
     * @note This class uses polymorphic memory resources (`std::pmr::memory_resource`).
     * @note Follows an STL-like design and supports iterators.
     */
    template <typename T>
    class stack {
    private:

        using stack_t = std::pmr::vector<T>;

        stack_t c;

    public:

        /**
         * @name Constructors
         * @{
         */

        /**
         * @brief Default constructor.
         *
         * @details Constructs a new stack, optionally associating it with a specific memory resource.
         *          If no memory resource is specified, the stack uses the default memory resource.
         *
         * @param [in, out] _resource (optional) Pointer to the memory resource to be used for memory allocations.
         *                            If not provided, the default memory resource is utilised.
         */
        constexpr stack([[maybe_unused]] std::pmr::memory_resource* _resource = std::pmr::get_default_resource()) : c(_resource) {}

        /**
         * @brief Constructs a new stack with a specified initial capacity.
         *
         * @details Creates a new stack object and reserves memory for at least `_capacity` elements.
         *          Optionally, associates the stack with a specific memory resource. If no memory resource
         *          is specified, the stack uses the default memory resource.
         *
         * @param [in] _capacity The number of elements to reserve storage for.
         * @param [in] _resource (optional) Pointer to the memory resource to be used for memory allocations.
         *                       If not provided, the default memory resource is utilised.
         */
        constexpr stack(size_t _capacity, [[maybe_unused]] std::pmr::memory_resource* _resource = std::pmr::get_default_resource()) : c(_resource) {
            c.reserve(_capacity);
        }

        /**
         * @brief Copy constructor.
         *
         * @details Constructs a new stack by copying the contents of another stack.
         *          The new stack will use the same memory resource as the source.
         *
         * @param [in] _other The stack to copy from.
         */
        constexpr stack(const stack& _other) : c(_other.c, _other.c.get_allocator().resource()) {}

        /**
         * @brief Copy assignment operator.
         *
         * @details Replaces the contents of this stack with the contents of another stack.
         *          The memory resource remains unchanged.
         *
         * @param [in] _other The stack to copy from.
         * @return Reference to this stack after assignment.
         */
        constexpr stack& operator=(const stack& _other) {
            if (this != &_other) {
                c = stack_t(_other.c, c.get_allocator().resource());
            }
            return *this;
        }

        /**
         * @brief Move constructor.
         *
         * @details Constructs a new stack by moving the contents of another stack.
         *          The new stack will steal the resources of the source stack.
         *
         * @param [in] _other The stack to move from.
         */
        constexpr stack(stack&& _other) noexcept : c(std::move(_other.c)) {}

        /**
         * @brief Move assignment operator.
         *
         * @details Replaces the contents of this stack with the contents of another stack.
         *          The memory resource remains unchanged, and the contents are moved.
         *
         * @param [in] _other The stack to move from.
         * @return Reference to this stack after assignment.
         */
        constexpr stack& operator=(stack&& _other) noexcept {
            if (this != &_other) {
                c = std::move(_other.c);
            }
            return *this;
        }

        /**
         * @}
         */

        /**
         * @brief Checks if the stack is empty.
         * @return `true` if the stack is empty, otherwise `false`.
         */
        [[maybe_unused, nodiscard]] HOT constexpr bool empty() const noexcept { return c.empty(); }

        /**
         * @brief Retrieves the number of elements currently stored in the stack.
         * @return The number of elements in the stack.
         */
        [[maybe_unused, nodiscard]] constexpr size_t size() const noexcept { return c.size(); }

        /**
         * @brief Provides access to the top element of the stack.
         *
         * @details Returns a reference to the most recently pushed element in the stack.
         *          The stack must not be empty when this method is called.
         *
         * @warning Invokes undefined behaviour if the queue is empty.
         * @return A reference to the top element of the stack.
         * @see empty()
         */
        [[maybe_unused, nodiscard]] HOT constexpr T& front() noexcept { return top(); }

        /**
         * @brief Provides access to the top element of the stack.
         *
         * @details Returns a reference to the most recently pushed element in the stack.
         *          The stack must not be empty when this method is called.
         *
         * @warning Invokes undefined behaviour if the queue is empty.
         * @return An immutable reference to the top element of the stack.
         * @see empty()
         */
        [[maybe_unused, nodiscard]] HOT constexpr const T& front() const noexcept { return top(); }

        /**
         * @brief Provides access to the top element of the stack.
         *
         * @details Returns a reference to the most recently pushed element in the stack.
         *          The stack must not be empty when this method is called.
         *
         * @warning Invokes undefined behaviour if the queue is empty.
         * @return A reference to the top element of the stack.
         * @see empty()
         */
        [[maybe_unused, nodiscard]] HOT constexpr T& top() noexcept { return c.back(); }

        /**
         * @brief Provides access to the top element of the stack.
         *
         * @details Returns a reference to the most recently pushed element in the stack.
         *          The stack must not be empty when this method is called.
         *
         * @warning Invokes undefined behaviour if the queue is empty.
         * @return An immutable reference to the top element of the stack.
         * @see empty()
         */
        [[maybe_unused, nodiscard]] HOT constexpr const T& top() const noexcept { return c.back(); }

        /**
         * @brief Adds an element to the stack.
         * @param [in] _value  A constant reference to the value to add to the stack.
         */
        [[maybe_unused]] HOT constexpr void push(const T& _value) { c.push(_value); }

        /**
         * @brief Adds an element to the stack, using move semantics.
         * @param [in] _value  A constant reference to the value to add to the stack.
         */
        [[maybe_unused]] HOT constexpr void push(T&& _value) { c.push(std::move(_value)); }

        /**
         * @brief Constructs an element in place, adding it to the stack.
         *
         * @details Constructs a new element at the top of the stack using the provided arguments.
         *          The arguments are perfectly forwarded to the constructor of the element,
         *          avoiding unnecessary copies or moves.
         *
         * @tparam Args Types of the arguments to be forwarded to the constructor of the element.
         *
         * @param [in] _args Variadic template parameters used to construct the new element.
         *                   These are perfectly forwarded to the element's constructor.
         */
        template <typename... Args>
        [[maybe_unused]] HOT constexpr void emplace(Args&&... _args) {
            c.emplace_back(std::forward<Args>(_args)...);
        }

        /**
         * @brief Pops an element from the stack.
         *
         * @details Removes the first element from the stack, reducing the size of the stack by one.
         *          As stackss maintain a FILO ordering, this will be the most recently added item.
         *
         * @warning Invokes undefined behaviour if the stack is empty.
         */
        [[maybe_unused]] HOT constexpr void pop() { c.pop_back(); }

        /**
         * @brief Clears all elements from the stack.
         *
         * @note The operation leaves the stack in a valid but empty state.
         *
         * @warning Ensure there are no pending references to the elements contained
         *          in the stack prior to invoking this method, as they will no longer
         *          be valid after the stack is cleared.
         */
        [[maybe_unused]] constexpr void clear() {
            c.clear();
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