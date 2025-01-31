/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef QUEUE_HPP
#define QUEUE_HPP

/**
 * @file queue.hpp
 */

#include <cstddef>
#include <queue>
#include <vector>
#include <memory_resource>

namespace chdr {

    /**
     * @nosubgrouping
     * @class queue
     * @brief Implementation of a First-In-First-Out (FIFO) container.
     *
     * @details Manages a collection of elements, maintaining a FIFO ordering.
     *
     * @tparam T  The type of elements stored in the queue.
     *
     * @note This class uses polymorphic memory resources (`std::pmr::memory_resource`).
     * @note Follows an STL-like design and supports iterators.
     */
    template <typename T>
    class queue {

    private:

        using queue_t = std::pmr::deque<T>;

        queue_t c;

    public:

        /**
         * @name Constructors
         * @{
         */

        /**
         * @brief Constructs a `queue` object.
         *
         * @details This constructor initialises the queue, with support for a custom memory resource.
         *          If no memory resource is explicitly provided, the default memory resource is used.
         *
         * @param [in, out] _resource A pointer to the memory resource to be used for
         *                       memory allocation. Defaults to the global default polymorphic memory resource.
         */
        constexpr queue(std::pmr::memory_resource* _resource = std::pmr::get_default_resource()) : c(_resource) {}

        ~queue() = default;

        /**
         * @brief Copy constructor.
         *
         * @details Constructs a new queue by copying the contents of another queue.
         *          The new queue will use the same memory resource as the source.
         *
         * @param [in] _other The queue to copy from.
         */
        constexpr queue(const queue& _other) : c(_other.c, _other.c.get_allocator().resource()) {}

        /**
         * @brief Copy assignment operator.
         *
         * @details Replaces the contents of this queue with the contents of another queue.
         *          The memory resource remains unchanged.
         *
         * @param [in] _other The queue to copy from.
         * @return Reference to this queue after assignment.
         */
        constexpr queue& operator=(const queue& _other) {
            if (this != &_other) {
                c = queue_t(_other.c, c.get_allocator().resource());
            }
            return *this;
        }

        /**
         * @brief Move constructor.
         *
         * @details Constructs a new queue by moving the contents of another queue.
         *          The new queue will steal the resources of the source queue.
         *
         * @param [in] _other The queue to move from.
         */
        constexpr queue(queue&& _other) noexcept : c(std::move(_other.c)) {}

        /**
         * @brief Move assignment operator.
         *
         * @details Replaces the contents of this queue with the contents of another queue.
         *          The memory resource remains unchanged, and the contents are moved.
         *
         * @param [in] _other The queue to move from.
         * @return Reference to this queue after assignment.
         */
        constexpr queue& operator=(queue&& _other) noexcept {
            if (this != &_other) {
                c = std::move(_other.c);
            }
            return *this;
        }

        /**
         * @}
         */

        /**
         * @brief Checks if the queue is empty.
         * @return `true` if the queue is empty, otherwise `false`.
         */
        [[maybe_unused, nodiscard]] HOT constexpr bool empty() const noexcept { return c.empty(); }

        /**
         * @brief Retrieves the number of elements currently stored in the queue.
         * @return The number of elements in the queue.
         */
        [[maybe_unused, nodiscard]] constexpr size_t size() const noexcept { return c.size(); }

        /**
         * @brief Provides access to the first element in the queue.
         *
         * @details Returns a reference to the element at the front of the queue.
         *          The queue must not be empty when this function is called.
         *
         * @return A reference to the first element in the queue.
         * @warning Invokes undefined behaviour if the queue is empty.
         *
         * @see empty()
         */
        [[maybe_unused, nodiscard]] HOT constexpr T& front() noexcept { return top(); }

        /**
         * @brief Provides access to the first element in the queue.
         *
         * @details Returns a reference to the element at the front of the queue.
         *          The queue must not be empty when this function is called.
         *
         * @return An immutable reference to the first element in the queue.
         * @warning Invokes undefined behaviour if the queue is empty.
         *
         * @see empty()
         */
        [[maybe_unused, nodiscard]] HOT constexpr const T& front() const noexcept { return top(); }

        /**
         * @brief Provides access to the first element in the queue.
         *
         * @details Returns a reference to the element at the front of the queue.
         *          The queue must not be empty when this function is called.
         *
         * @return A reference to the first element in the queue.
         * @warning Invokes undefined behaviour if the queue is empty.
         *
         * @see empty()
         */
        [[maybe_unused, nodiscard]] HOT constexpr T& top() noexcept { return c.front(); }

        /**
         * @brief Provides access to the first element in the queue.
         *
         * @details Returns a reference to the element at the front of the queue.
         *          The queue must not be empty when this function is called.
         *
         * @return An immutable reference to the first element in the queue.
         * @warning Invokes undefined behaviour if the queue is empty.
         *
         * @see empty()
         */
        [[maybe_unused, nodiscard]] HOT constexpr const T& top() const noexcept { return c.front(); }

        /**
         * @brief Adds an element to the queue.
         * @param [in] _value  A constant reference to the value to add to the queue.
         */
        [[maybe_unused]] HOT constexpr void push(const T& _value) { c.push(_value); }

        /**
         * @brief Adds an element to the queue, using move semantics.
         * @param [in] _value  A constant reference to the value to add to the queue.
         */
        [[maybe_unused]] HOT constexpr void push(T&& _value) { c.push(std::move(_value)); }

        /**
         * @brief Constructs an element in place, adding it to the queue.
         *
         * @details Constructs a new element at the back of the queue using the provided arguments.
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
         * @brief Pops an element from the queue.
         *
         * @details Removes the first element from the queue, reducing the size of the queue by one.
         *          As queues maintain a FIFO ordering, this will be the least recently added item.
         *
         * @warning Invokes undefined behaviour if the queue is empty.
         */
        [[maybe_unused]] HOT constexpr void pop() { c.pop_front(); }

        /**
         * @brief Clears all elements from the queue.
         *
         * @note The operation leaves the queue in a valid but empty state.
         *
         * @warning Ensure there are no pending references to the elements contained
         *          in the queue prior to invoking this method, as they will no longer
         *          be valid after the queue is cleared.
         */
        [[maybe_unused]] constexpr void clear() {
            c = std::move(decltype(c){});
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

#endif //QUEUE_HPP