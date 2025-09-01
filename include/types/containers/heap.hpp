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

/**
 * @file heap.hpp
 */

#include <cstddef>
#include <functional>
#include <vector>

// ReSharper disable once CppUnusedIncludeDirective
#include <memory_resource> // NOLINT(*-include-cleaner)

// ReSharper disable once CppUnusedIncludeDirective
#include "../../utils/intrinsics.hpp" // NOLINT(*-include-cleaner)

namespace chdr {

    /**
     * @nosubgrouping
     * @brief A generic heap implementation with configurable properties.
     *
     * @details Implements a heap data structure, allowing priority-based storage and retrieval of elements.
     *          It supports custom comparator functions and uses a container that manages its items internally.
     *          The heap is optimised for performance and includes methods for managing its structure while
     *          maintaining the heap property.\n\n
     *          Dimensionality of the heap can be customised through the template parameters.
     *
     * Further Reading:
     * - <a href="https://en.wikipedia.org/wiki/Heap_(data_structure)">Wikipedia Article</a>
     * - <a href="https://stackoverflow.com/questions/6531543/efficient-implementation-of-binary-heaps">Stack Overflow — Efficient Implementation of Binary Heaps</a>
     * - Skiena, S. S., 2020. The Algorithm Design Manual [online]. Cham: Springer International Publishing. 81-93, Available from: http://link.springer.com/10.1007/978-3-030-54256-6 [Accessed 28 Jan 2025].
     *
     * @tparam T The type of elements contained in the heap.
     * @tparam Container The type of container used to store the heap elements (defaulted to a PMR vector).
     * @tparam Compare A callable object that defines the order relationship between elements.
     * @tparam Kd The dimension of the heap (defaulted to binary, i.e., Kd=2). Must be ≥ 2.
     * @note This class uses polymorphic memory resources (`std::pmr::memory_resource`).
     * @note Follows an STL-like design and supports iterators.
     */
    template <typename T, typename Compare = std::less<T>, typename Container = std::pmr::vector<T>, size_t Kd = 2U>
    class heap {

        static_assert(Kd >= 2U, "Heap dimensionality must be greater than or equal to 2.");

    protected:
        Container c;
        Compare   comp;

    private:

        /**
         * @brief Calculates the index of a given item within the heap container.
         *
         * @details This function determines the zero-based index of an item in the container `c` by calculating the offset
         *          from the address of a reference item `_item` to the address of the container's super element (`c[1U]`).
         *          It is intended to be used exclusively for determining positions in an internal heap structure.
         *
         * @param _item [in] The item whose index within the heap is to be determined.
         * @return size_t The zero-based index of the provided item within the heap container.
         */
        [[nodiscard]] HOT constexpr size_t index_of(const T& _item) const noexcept {
            return static_cast<size_t>(&(_item) - &(c[0U]));
        }

    public:

        /**
         * @name Constructors
         * @{
         */

        /**
         * @brief Constructs a heap container with an optional memory resource.
         *
         * @details Initialises the heap container `c` with a memory resource.
         *          By default, it uses the global default memory resource. Additionally, the
         *          container is resized to include one uninitialised super element, which serves
         *          as an internal placeholder within the heap structure.
         *
         * @param [in, out] _resource (optional) A pointer to a polymorphic memory resource to be used
         *        for managing memory allocations within the heap. If not provided, the default
         *        memory resource is used.
         */
        [[maybe_unused, nodiscard]] explicit heap(std::pmr::memory_resource* _resource = std::pmr::get_default_resource()) : c(_resource) {
            c.resize(1U); // Add uninitialised super element.
        }

        /**
         * @brief Constructs a heap container with a specified initial capacity and memory resource.
         *
         * @details Initialises the heap container, setting its capacity to the specified `_capacity` plus one.
         *          It uses the provided polymorphic memory resource to manage heap allocations.
         *          A super element placeholder is added as the first element of the container to
         *          reserve space for internal heap operations.
         *
         * @param _capacity The desired capacity of the heap container.
         * @param [in, out] _resource (optional) A pointer to a polymorphic memory resource for managing memory allocations.
         *                                       If not provided, the default polymorphic memory resource is used.
         */
        [[maybe_unused, nodiscard]] explicit heap(size_t _capacity, std::pmr::memory_resource* _resource = std::pmr::get_default_resource()) : c(_resource) {
            c.reserve(utils::min(_capacity, std::numeric_limits<size_t>::max() - 1U) + 1U);
            c.emplace_back(); // Add uninitialised super element.
        }

        /**
         * @brief Default destructor.
         */
        ~heap() = default;

        /**
         * @brief Copy constructor.
         *
         * @details Creates a deep copy of the given heap, including its internal container and comparator.
         *
         * @param _other The heap to copy from.
         */
        heap(const heap& _other) : c(_other.c), comp(_other.comp) {}

        /**
         * @brief Copy assignment operator.
         *
         * @details Assigns another heap to this one by performing a deep copy of the other heap's internal
         * container and comparator.
         *
         * @param _other The heap to copy from.
         * @return Reference to this heap after the assignment.
         */
        heap& operator=(const heap& _other) {
            if (this != &_other) {
                c    = _other.c;
                comp = _other.comp;
            }
            return *this;
        }

        /**
         * @brief Move constructor.
         *
         * @details Transfers ownership of the resources of the given heap to this one,
         * leaving the other heap in a valid but unspecified state.
         *
         * @param _other The heap to move from.
         */
        heap(heap&& _other) noexcept : c(std::move(_other.c)), comp(std::move(_other.comp)) {}

        /**
         * @brief Move assignment operator.
         *
         * @details Transfers ownership of the resources of the given heap to this one,
         * leaving the other heap in a valid but unspecified state.
         *
         * @param _other The heap to move from.
         * @return Reference to this heap after the assignment.
         */
        heap& operator=(heap&& _other) noexcept {
            if (this != &_other) {
                c    = std::move(_other.c);
                comp = std::move(_other.comp);
            }
            return *this;
        }

        /**
         * @brief Constructs a heap from a collection of elements.
         *
         * @details Initialises the heap with a copy of the elements from the provided container.
         *          Uses the fast heap sort algorithm.
         *
         * @param _container [in] A container of elements to initialise the heap. Its allocator provides the memory resource
         *                        used for this heap's internal container.
         */
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

        /**
         * @brief Constructs a heap from a collection of elements with move semantics.
         *
         * @details Initialises the heap with by moving the elements from the provided container.
         *          Uses the fast heap sort algorithm.
         *
         * @param _container [in] A container of elements to initialise the heap. Its allocator provides the memory resource
         *                        used for this heap's internal container.
         */
        explicit heap(Container&& _container) : c(_container.get_allocator().resource()), comp() {

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

        /**
         * @}
         */

        /**
         * @brief Checks if the heap is empty.
         * @return `true` if the heap is empty, otherwise `false`.
         */
        [[maybe_unused, nodiscard]] HOT constexpr bool empty() const noexcept { return size() == 0U; }

        /**
         * @brief Retrieves the number of elements currently stored in the heap.
         * @return The number of elements in the heap.
         */
        [[maybe_unused, nodiscard]] constexpr size_t size() const noexcept { return utils::max(c.size(), static_cast<size_t>(1U)) - static_cast<size_t>(1U); }

        /**
         * @brief Retrieves the maximum number of elements that can be held by the container without reallocating memory.
         * @return size_t The maximum number of elements that the container can accommodate without reallocating.
         */
        [[maybe_unused, nodiscard]] constexpr size_t capacity() const noexcept { return c.capacity(); }

        /**
         * @brief Retrieves a reference to the front element of the heap.
         * @pre The heap must not be empty before calling this function.
         * @return T& A reference to the front element in the heap.
         */
        [[maybe_unused, nodiscard]] HOT constexpr T& front() noexcept { return top(); }

        /**
         * @brief Retrieves a constant reference to the front element of the heap.
         * @pre The heap must not be empty before calling this function.
         * @return T& A reference to the front element in the heap.
         */
        [[maybe_unused, nodiscard]] HOT constexpr const T& front() const noexcept { return top(); }

        /**
         * @brief Retrieves a reference to the top element of the heap.
         * @pre The heap must not be empty before calling this function.
         * @return T& A reference to the top element of the heap.
         */
        [[maybe_unused, nodiscard]] HOT constexpr T& top() noexcept {

            assert(!empty() && "Heap is empty");

            if constexpr (std::is_pointer_v<T>) {
                return reinterpret_cast<T&>(begin().base());
            }
            else {
                return *begin();
            }
        }

        /**
         * @brief Retrieves a constant element to the top element of the heap.
         * @pre The heap must not be empty before calling this function.
         * @return T& A reference to the top element of the heap.
         */
        [[maybe_unused, nodiscard]] HOT constexpr const T& top() const noexcept {

            assert(!empty() && "Heap is empty");

            if constexpr (std::is_pointer_v<T>) {
                return reinterpret_cast<T&>(begin().base());
            }
            else {
                return *begin();
            }
        }

        /**
         * @brief Retrieves a reference to the last element in the heap container.
         * @pre The heap must not be empty before calling this function.
         * @return T& A constant reference to the last element in the heap.
         */
        [[maybe_unused, nodiscard]] HOT constexpr T& back() noexcept {

            assert(!empty() && "Heap is empty");

            if constexpr (std::is_pointer_v<T>) {
                return reinterpret_cast<T&>(end().base());
            }
            else {
                return *end();
            }
        }

        /**
         * @brief Retrieves a constant reference to the last element in the heap container.
         * @pre The heap must not be empty before calling this function.
         * @return T& A constant reference to the last element in the heap.
         */
        [[maybe_unused, nodiscard]] HOT constexpr const T& back() const noexcept {

            assert(!empty() && "Heap is empty");

            if constexpr (std::is_pointer_v<T>) {
                return reinterpret_cast<T&>(end().base());
            }
            else {
                return *end();
            }
        }

        /**
         * @brief Adds an item to the heap.
         * @param [in] _item Item to add to the heap.
         */
        [[maybe_unused]] HOT constexpr void enqueue(const T& _item) { push(_item); }

        /**
         * @brief Adds a new element to the heap container using perfect forwarding.
         * @tparam Args The variadic template parameter pack representing the types of the arguments passed
         *              to construct the new element.
         * @param _args The arguments being forwarded to construct an element in the heap.
         */
        template <class... Args>
        [[maybe_unused]] HOT constexpr void enqueue(Args&&... _args) { emplace(std::forward<Args>(_args)...); }

        /**
         * @brief Adds an item to the heap.
         * @param [in] _item Item to add to the heap.
         */
        [[maybe_unused]] HOT constexpr void push(const T& _item) {
            c.push_back(_item);
            sort_up(c.back());
        }

        /**
         * @brief Adds an item to the heap using move semantics.
         * @param [in] _item Item to move into the heap.
         */
        [[maybe_unused]] HOT constexpr void push(T&& _item) {
            c.push_back(std::move(_item));
            sort_up(c.back());
        }

        /**
         * @brief Adds a new element to the heap container using perfect forwarding.
         * @tparam Args The variadic template parameter pack representing the types of the arguments passed
         *              to construct the new element.
         * @param _args The arguments being forwarded to construct an element in the heap.
         */
        template <class... Args>
        [[maybe_unused]] HOT constexpr void emplace(Args&&... _args) {
            c.emplace_back(std::forward<Args>(_args)...);
            sort_up(c.back());
        }

        /**
         * @brief Adds an item to the heap. Does not sort the heap.
         * @param [in] _item Item to add to the heap.
         * @warning Not sorting the heap may violate the heap property.
         */
        [[maybe_unused]] HOT constexpr void enqueue_nosort(const T& _item) { push_nosort(_item); }

        /**
         * @brief Adds a new element to the heap container using perfect forwarding. Does not sort the heap.
         * @tparam Args The variadic template parameter pack representing the types of the arguments passed
         *              to construct the new element.
         * @param _args The arguments being forwarded to construct an element in the heap.
         * @warning Not sorting the heap may violate the heap property.
         */
        template <class... Args>
        [[maybe_unused]] HOT constexpr void enqueue_nosort(Args&&... _args) { emplace_nosort(std::forward<Args>(_args)...); }

        /**
         * @brief Adds an item to the heap. Does not sort the heap.
         * @param [in] _item Item to add to the heap.
         * @warning Not sorting the heap may violate the heap property.
         */
        [[maybe_unused]] HOT constexpr void push_nosort(const T& _item) { c.push_back(_item); }

        /**
         * @brief Adds a new element to the heap container using move semantics.
         * @param [in] _item Item to move into the heap.
         * @warning Not sorting the heap may violate the heap property.
         */
        [[maybe_unused]] HOT constexpr void push_nosort(T&& _item) { c.push_back(std::move(_item)); }

        /**
         * @brief Adds a new element to the heap container using perfect forwarding. Does not sort the heap.
         * @tparam Args The variadic template parameter pack representing the types of the arguments passed
         *              to construct the new element.
         * @param _args The arguments being forwarded to construct an element in the heap.
         * @warning Not sorting the heap may violate the heap property.
         */
        template <class... Args>
        [[maybe_unused]] HOT constexpr void emplace_nosort(Args&&... _args) { c.emplace_back(std::forward<Args>(_args)...); }

        /**
         * @brief Removes a specified item from the heap, restoring the heap property after removal.
         *
         * @details This function erases a given item from the heap container, ensuring the heap structure's integrity
         *          is preserved. If the item to be removed is the last item, it is simply removed. Otherwise, the last
         *          element replaces the erased item and the heap property is restored through appropriate heap adjustments.
         *
         * @param _item [in] The item to be removed from the heap container.
         */
        [[maybe_unused]] constexpr void erase(const T& _item) noexcept {

            assert(!empty() && "Heap is empty");

            auto i = index_of(_item);

            assert(i < c.size() && "(Out of Bounds) Item does not exist in Heap.");

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

        /**
         * @brief Removes and returns the top element from the heap.
         *
         * @details This function extracts the element at the top of the heap, ensuring the heap's internal structure
         *          remains valid after the operation. The top element is replaced with the last element in the
         *          container before being removed, and the internal order of the heap is restored using `sort_down`.
         *
         * @return T The element extracted from the top of the heap.
         *
         * @pre The heap must not be empty before calling this function.
         * @remark This function assumes the heap is not empty.
         * @warning Behaviour is undefined if called on an empty heap.
         */
        [[maybe_unused, nodiscard]] constexpr T dequeue() noexcept {
            assert(!empty() && "Heap is empty");

            T result(std::move(top()));

            if (size() > 0U) {
                c[1U] = std::move(c.back());
            }
            c.pop_back();

            sort_down(c[1U]);

            return result;
        }

        /**
         * @brief Removes the top element from the heap and maintains the heap property.
         *
         * @details This function removes the element at the top of the heap (i.e., the highest priority element),
         *          swaps it with the last element in the container, and then adjusts the heap to preserve its structure
         *          and maintain the heap property.
        *
         * @pre The heap must not be empty before calling this function.
         * @remark This function assumes the heap is not empty.
         * @warning Behaviour is undefined if called on an empty heap.
         */
        [[maybe_unused]] HOT constexpr void pop() noexcept {
            assert(!empty() && "Heap is empty");

            if (size() > 0U) {
                c[1U] = std::move(c.back());
            }
            c.pop_back();

            sort_down(c[1U]);
        }

        /**
         * @brief Removes the last element from the heap.
         *
         * @details This function removes the element at the back of the heap (i.e., the lowest priority element).
         *
         * @pre The heap must not be empty before calling this function.
         * @remark This function assumes the heap is not empty.
         * @warning Behaviour is undefined if called on an empty heap.
         */
        [[maybe_unused]] HOT constexpr void pop_back() noexcept {
            assert(!empty() && "Heap is empty");
            c.pop_back();
        }

        /**
         * @brief Adjusts the position of an item in the heap by moving it upwards towards the root.
         *
         * @details This function is used to maintain the heap's structural and ordering properties
         *          when an item's position may have been compromised.
         *
         * @param _item [in] The item to be sorted upwards within the heap. The item's new position
         *                    will adhere to the heap's rules after execution.
         */
        HOT constexpr void sort_up(const T& _item) noexcept {

            if (size() > 1U) {

                auto i = index_of(_item);

                assert(i < c.size() && "(Out of Bounds) Item does not exist in Heap.");

                auto value_to_insert = std::move(c[i]);

                while (i > 1U) {
                    const auto p = i / Kd;

                    if (comp(c[p], value_to_insert)) {
                        c[i] = std::move(c[p]);
                        i = p;
                    }
                    else {
                        break;
                    }
                }

                c[i] = std::move(value_to_insert);
            }
        }

        /**
         * @brief Reorganises the heap by moving a specified item downward to maintain the heap property.
         *
         * @details This function moves the specified item downward within the heap structure to restore the heap's ordering
         *          property. It is typically used after a modification or removal operation that causes the heap to become
         *          unbalanced.
         *
         * @param _item [in] The item to be moved downward within the heap to restore the heap property.
         */
        HOT constexpr void sort_down(const T& _item) noexcept {

            if (size() > 1U) {

                auto i = index_of(_item);
                assert(i < c.size() && "(Out of Bounds) Item does not exist in Heap.");

                auto value_to_insert = std::move(c[i]); // Save value to reposition.

                while (true) {

                    const size_t c0 = i * Kd;         // First child index
                    const size_t cn = c0 + (Kd - 1U); // Last child index in range

                    if (c0 < c.size()) {
                        PREFETCH(&c[c0], _MM_HINT_NTA);
                        PREFETCH(&c[cn], _MM_HINT_NTA);

                        // Find best candidate child to swap with:
                        size_t best_child = c0;
                        for (size_t j = c0 + 1U; j <= cn && j < c.size(); ++j) {
                            if (comp(c[best_child], c[j])) {
                                best_child = j;
                            }
                        }

                        // If value to insert is less favorable than the best child:
                        if (comp(value_to_insert, c[best_child])) {
                            c[i] = std::move(c[best_child]);
                            i = best_child;
                            continue;
                        }
                    }

                    c[i] = std::move(value_to_insert);
                    break;
                }
            }
        }

        /**
         * @brief Checks whether the specified item exists in the heap container.
         *
         * @details This function evaluates if a given item `_item` is present within the heap by determining its index
         *          and verifying its equality with the corresponding element in the container `c`. The function ensures
         *          that the heap is not empty before proceeding.
         *
         * @param _item [in] A reference to the item whose presence in the heap is to be checked.
         * @return bool True if the item exists in the heap container; otherwise, false.
         */
        [[maybe_unused, nodiscard]] constexpr bool contains(T& _item) noexcept {

            bool result = !empty();
            if (result) {
                const auto& i = index_of(_item);
                result = i < c.size() && _item == c[i];
            }

            return result;
        }

        /**
         * @brief Reserves memory for at least the specified number of elements in the container.
         *
         * @details This function increases the capacity of the container to the specified value,
         *          ensuring that the container can hold at least the provided number of elements
         *          without requiring a reallocation. If the requested capacity is less
         *          than the current capacity, the call has no effect.
         *          This function does not change the size of the container.
         *
         * @param _capacity The minimum number of elements the container should be able to hold.
         */
        [[maybe_unused]] constexpr void reserve(size_t _capacity) {
            c.reserve(_capacity);
        }

        /**
         * @brief Swaps the contents of the current heap instance with another heap instance.
         *
         * @details Exchanges the contents and comparator of the current heap object
         *          with those of another heap object.
         *
         * @param _other [in, out] The heap instance to swap with the current heap instance.
         */
        [[maybe_unused]] constexpr void swap(heap& _other) noexcept {
            if (this != &_other) {
                std::swap(c,    _other.c);
                std::swap(comp, _other.comp);
            }
        }

        /**
         * @brief Clears all elements from the heap container.
         *
         * @details This function removes all elements from the heap aside from the super element,
         *          leaving it empty. After calling this function, the heap structure
         *          will be entirely reset. This action does not deallocate memory.
         */
        [[maybe_unused]] constexpr void clear() noexcept { c.erase(begin(), end()); }

        /**
         * @brief Reduces the memory usage of the container by deallocating unused memory.
         *
         * @details This function requests the container to release any excess capacity it might hold beyond its current size.
         *          It is particularly useful for optimising memory usage after significant removal of elements.
         *          The operation does not alter the size or the elements of the container.
         */
        [[maybe_unused]] constexpr void shrink_to_fit() noexcept { c.shrink_to_fit(); }

        /**
         * @brief Provides access to an element at a specified index within the heap container.
         *
         * @details This function retrieves a reference to an element located at the given zero-based index
         *          in the heap structure.
         *
         * @param _index The zero-based index of the element to retrieve.
         * @return const T& A constant reference to the element at the specified index within the heap container.
         * @see operator[]()
         */
        [[maybe_unused, nodiscard]] constexpr const T& at(size_t _index) const {
            return c.at(_index + 1U);
        }

        /**
         * @brief Wipes the heap, leaving it in an invalid state.
         *
         * @details This function completely wipes the internal container of the heap,
         *          leaving it free of all elements, including the super element.
         *
         * @warning Running this code leaves the heap in an invalid state. It should not be used afterwards.
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

        /**
         * @brief Accesses an element in the heap container by its zero-based index.
         *
         * @details This operator provides read-only access to the `T` element at the specified index within the heap's container.
         *          The internal structure assumes a one-based indexing system due to the heap's layout, so the user-provided index
         *          is incremented by one before accessing the container. It is intended for accessing heap elements in a constant
         *          time complexity.
         *
         * @param [in] _index The zero-based index of the element to access within the heap container.
         * @return const T& A constant reference to the element located at the specified index in the heap container.
         */
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

        /**
         * @brief Finds the first element in the heap not less than the specified value.
         *
         * @details This function performs a search within the heap structure to locate the first
         *          element that is not less than the given value `_value`. The search starts
         *          from the root node and traverses the heap, comparing elements based on
         *          the provided comparator and utilising the K-dimensional structure.
         *          If no such element is found, a `end()` iterator is returned.
         *
         * @param _value [in] The value to find the lower bound for within the heap container.
         * @return iterator_t An iterator pointing to the first element not less than `_value`,
         *         or `end()` if no such element exists.
         */
        [[maybe_unused, nodiscard]] constexpr iterator_t lower_bounds(const T& _value) noexcept {

            if (!empty()) {

                for (size_t i = 1U; i < c.size();) { // Start from index 1 to skip the super element.

                    if (!comp(c[i], _value)) {
                        return c.begin() + i; // Return the first element not less than the given value.
                    }

                    // Iterate through child nodes based on the Kd dimension.
                    const size_t child_start = i * Kd;
                    const size_t child_end   = std::min(child_start + Kd, c.size());

                    bool traversed = false;
                    for (size_t j = child_start; j < child_end; ++j) {
                        if (j < c.size() && !comp(c[j], _value)) {
                            i = j; // Traverse to the first valid child node.
                            traversed = true;
                            break;
                        }
                    }

                    if (!traversed) {
                        break; // Exit loop if no valid traversal path exists.
                    }
                }
            }

            return c.end(); // Return end if no matching element is found.
        }

        /**
         * @brief Finds the first element in the heap greater than the specified value.
         *
         * @details This function performs a search within the heap structure to locate the first
         *          element that is greater than the given value `_value`. The search starts
         *          from the root node and traverses the heap, comparing elements based on
         *          the provided comparator and utilising the K-dimensional structure.
         *          If no such element is found, a `end()` iterator is returned.
         *
         * @param _value [in] The value to find the upper bound for within the heap container.
         * @return iterator_t An iterator pointing to the first element greater than `_value`,
         *         or `end()` if no such element exists.
         */
        [[maybe_unused, nodiscard]] constexpr iterator_t upper_bounds(const T& _value) noexcept {

            if (!empty()) {

                for (size_t i = 1U; i < c.size();) { // Start from index 1 to skip the super element.

                    if (comp(_value, c[i])) {
                        return c.begin() + i; // Return the first element greater than the given value.
                    }

                    // Iterate through child nodes based on the Kd dimension.
                    const size_t child_start = i * Kd;
                    const size_t child_end   = std::min(child_start + Kd, c.size());

                    bool traversed = false;
                    for (size_t j = child_start; j < child_end; ++j) {
                        if (j < c.size() && comp(_value, c[j])) {
                            i = j; // Traverse to the first valid child node.
                            traversed = true;
                            break;
                        }
                    }

                    if (!traversed) {
                        break; // Exit loop if no valid traversal path exists.
                    }
                }
            }

            return c.end(); // Return end if no matching element is found.
        }

        /**
         * @brief Inserts an element into the heap while maintaining the heap property.
         *
         * @param _position The iterator position where the element should be inserted.
         * @param _value The value to insert into the heap.
         * @return iterator_t An iterator pointing to the inserted element.
         */
        constexpr iterator_t insert(iterator_t _position, const T& _value) {
            auto distance_from_start = std::distance(c.begin(), _position) - 1U; // Adjust for super element.
            c.insert(_position, _value);
            sort_up(c[distance_from_start]);
            return c.begin() + distance_from_start + 1U; // Return iterator to inserted element
        }

        /**
         * @brief Moves an element into the heap while maintaining the heap property.
         *
         * @param _position The iterator position where the element should be moved.
         * @param _value The value to insert into the heap.
         * @return iterator_t An iterator pointing to the inserted element.
         */
        constexpr iterator_t insert(iterator_t _position, T&& _value) {
            auto distance_from_start = std::distance(c.begin(), _position) - 1U; // Adjust for super element.
            c.insert(_position, std::move(_value));
            sort_up(c[distance_from_start]);
            return c.begin() + distance_from_start + 1U; // Return iterator to inserted element
        }

        /**
         * @brief Removes an element from the heap using an iterator while maintaining the heap property.
         *
         * @param _position The iterator position of the element to remove.
         * @return iterator_t An iterator pointing to the element that follows the removed one.
         */
        constexpr iterator_t erase(iterator_t _position) {

            const auto distance_from_start = static_cast<unsigned>(std::distance(c.begin(), _position));
            const auto size_before = size();

            if (distance_from_start < size() + 1U) { // Offset for super element.

                if (distance_from_start == size()) {
                    c.pop_back(); // Remove if last element.
                }
                else {

                    c[distance_from_start] = std::move(c.back());
                    c.pop_back();
                    if (size_before > 1U) {
                        if (distance_from_start > 1U && comp(c[distance_from_start], c[distance_from_start / Kd])) {
                            sort_up(c[distance_from_start]);
                        }
                        else {
                            sort_down(c[distance_from_start]);
                        }
                    }
                }
            }

            return c.begin() + distance_from_start; // Return iterator to next element after erasure
        }

        /**
         * @brief Inserts a range of elements into the heap.
         *
         * @tparam InputIt An input iterator type.
         * @param _first The beginning of the range to insert.
         * @param _last The end of the range to insert.
         */
        template <typename InputIt>
        constexpr void insert(InputIt _first, InputIt _last) {
            for (InputIt it = _first; it != _last; ++it) {
                c.push_back(*it);
                sort_up(c.back());
            }
        }

        /**
         * @brief Removes all elements in a given range from the heap while maintaining the heap property.
         *
         * @param _first Iterator to the first element of the range.
         * @param _last Iterator to the element past the last one to erase.
         */
        constexpr void erase(iterator_t _first, iterator_t _last) {
            for (auto it = _first; it != _last;) {
                it = erase(it); // Keep erasing until the last element is removed.
            }
        }

        /**
         * @brief Returns the internal container representing the heap.
         */
        constexpr const auto& __internal_container() {
            return c;
        }

        [[maybe_unused, nodiscard]] constexpr iterator_t        begin()       noexcept { return c.begin()  + 1U; }
        [[maybe_unused, nodiscard]] constexpr const_iterator_t  begin() const noexcept { return c.begin()  + 1U; }
        [[maybe_unused, nodiscard]] constexpr const_iterator_t cbegin() const noexcept { return c.cbegin() + 1U; }

        [[maybe_unused, nodiscard]] constexpr iterator_t        end()       noexcept { return c.end();  }
        [[maybe_unused, nodiscard]] constexpr const_iterator_t  end() const noexcept { return c.end();  }
        [[maybe_unused, nodiscard]] constexpr const_iterator_t cend() const noexcept { return c.cend(); }

        [[maybe_unused, nodiscard]] constexpr reverse_iterator_t        rbegin()       noexcept { return c.rbegin();  }
        [[maybe_unused, nodiscard]] constexpr const_reverse_iterator_t  rbegin() const noexcept { return c.rbegin();  }
        [[maybe_unused, nodiscard]] constexpr const_reverse_iterator_t crbegin() const noexcept { return c.crbegin(); }

        [[maybe_unused, nodiscard]] constexpr reverse_iterator_t        rend()       noexcept { return c.rend();  }
        [[maybe_unused, nodiscard]] constexpr const_reverse_iterator_t  rend() const noexcept { return c.rend();  }
        [[maybe_unused, nodiscard]] constexpr const_reverse_iterator_t crend() const noexcept { return c.crend(); }

    };

} //chdr

#endif