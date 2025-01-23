/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_EXISTENCE_SET_HPP
#define CHDR_EXISTENCE_SET_HPP

#include <cstddef>
#include <initializer_list>
#include <memory_resource>
#include <type_traits>
#include <vector>

#include "../../utils/utils.hpp"

namespace chdr {

    /** @brief One bit represents each item. */
    struct low_memory_usage final {};

    /** @brief One byte represents each item. */
    struct high_performance final {};

    /**
     * @nosubgrouping
     * @class existence_set
     * @brief Represents a container for tracking the existence of elements.
     *
     * @details The `existence_set` is a specialised data structure designed to efficiently track
     *          the existence of elements without storing their original values. \n\n
     *          It has constant lookup, insertion, and removal times, storing data in a dense,
     *          contiguous memory layout that improves the cache locality of stored elements. \n\n
     *          Due to its dense structure, the existence set experiences an increased worst-case
     *          memory complexity. However, as it is non-owning, it often uses less memory in
     *          monotonic situations than its sparse counterparts. \n\n
     *          Memory efficiency and performance are customisable through specifying the number
     *          of bits used with the provided template parameter.
     *
     * @tparam layout_t Memory layout strategy for the set:
     *                  - `low_memory_usage`: Minimises memory usage (One bit per item).
     *                  - `high_performance`: Maximises performance with increased memory usage (One byte per item).
     *
     * @note This class uses polymorphic memory resources (`std::pmr::memory_resource`)
     *       to provide fine-grained control over memory allocation.
     *
     * @see low_memory_usage
     * @see high_performance
     */
    template <typename layout_t = high_performance>
    class existence_set {

        // Validate the template parameter.
        static_assert(
            std::is_same_v<layout_t, low_memory_usage> ||
            std::is_same_v<layout_t, high_performance>,
            "layout_t must be one of the following: low_memory_usage, high_performance"
        );

        /**
         * @typedef boolean_t
         * @brief Boolean type based on the specified layout_t.
         * @details If the specified layout_t is `lowest_memory_usage`, the corresponding boolean type is `bool`.
         *          Otherwise, for `high_performance`, the boolean type is `char`.
         */
        using boolean_t = std::conditional_t<std::is_same_v<layout_t, low_memory_usage>, bool, char>;

    private:
        /**
         * @brief Storage for tracking existence.
         * @details This structure tracks whether elements are present or absent
         *          without storing their actual values.
         *
         * @note Currently uses a polymorphic memory resource-enabled vector.
         *       Future versions may support other container types for added flexibility and constexpr support.
         */
        std::pmr::vector<boolean_t> c;

        /**
         * @brief Enables a hash in the set.
         * @details Marks a specific hash as present in the set.
         * @note This operation resizes the set automatically if necessary.
         * @param _hash The hash value to enable.
         */
        constexpr void enable(size_t _hash) {

            if (_hash >= c.size()) {
                resize(_hash + 1U);
            }

            /*
             * Suppress GCC-specific warnings about potential string overflow in this section of code.
             *
             * As 'boolean_t' can be a valid alias for 'char' type, linting tools occasionally
             * misidentify this code as string manipulation.
             */
#if defined(GCC)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstringop-overflow"
#endif

            c[_hash] = static_cast<boolean_t>(true);

#if defined(GCC)
#pragma GCC diagnostic pop
#endif
        }

        /**
         * @brief Disable a hash in the set.
         * @details Marks a specific hash as absent in the set.
         * @note This operation does not reduce the size of the set.
         * @param _hash The hash value to 'turn off'.
         */
        constexpr void disable(size_t _hash) noexcept {
            if (_hash < c.size()) {
                c[_hash] = static_cast<boolean_t>(false);
            }
        }

    public:

        /**
         * @name Constructors
         * @{
         */

        /**
         * @brief Construct an existence_set.
         *
         * @remarks This constructor allows the existence_set to allocate memory using a specified
         *          polymorphic memory resource. If no resource is provided, the default memory
         *          resource is used.
         *
         * @param [in, out] _resource (optional) A pointer to a std::pmr::memory_resource object
         * that will be used for memory allocation. If not provided, the default memory
         * resource is used.
         */
        [[maybe_unused]] constexpr existence_set(std::pmr::memory_resource* _resource = std::pmr::get_default_resource()) noexcept : c(_resource) {}

        /**
         * @brief Constructs an existence_set with the specified capacity.
         *
         * @remarks This constructor allows the existence_set to allocate memory using a specified
         *          polymorphic memory resource. If no resource is provided, the default memory
         *          resource is used.
         *
         * @param _capacity The initial number of elements to reserve space for.
         * @param [in, out] _resource (optional) A pointer to the memory resource to use for allocations.
         *                      If no custom resource is provided, the default memory resource is used.
         */
        [[maybe_unused]] constexpr explicit existence_set(size_t _capacity, std::pmr::memory_resource* _resource = std::pmr::get_default_resource()) : c(_resource) {
            reserve(_capacity);
        }

        /**
         * @brief Constructs an existence_set with initial items.
         *
         * @details This constructor initialises the existence_set using a list of hashes.
         *          Each hash is inserted into the existence set during initialisation.
         *
         * @note Duplicate hashes will be merged into single entries.
         *
         * @remarks This constructor allows the existence_set to allocate memory using a specified
         *          polymorphic memory resource. If no resource is provided, the default memory
         *          resource is used.
         *
         * @param [in] _items The items to initialise the existence_set with, given as an initializer list.
         * @param [in, out] _resource (optional) Pointer to the memory resource to use for the internal memory management.
         *                        Defaults to std::pmr::get_default_resource().
         */
        [[maybe_unused]] constexpr existence_set(const std::initializer_list<size_t>& _items, std::pmr::memory_resource* _resource = std::pmr::get_default_resource()) : c(_resource) {

            reserve(_items.size());

            for (const auto& item : _items) {
                push(item);
            }
        }

        /**
         * @}
         */

        /**
         * @brief Allocates additional capacity to accommodate the hash, ensuring an increase
         *        in storage is within the specified limits.
         *
         * @details This function increases the capacity of the container when the current capacity
         *          is insufficient to include the specified hash value. The increment value specifies
         *          how much the capacity should be increased by. An optional maximum increment
         *          value can be provided to ensure the increase does not exceed this limit.
         *
         * @param _hash The hash value that requires storage. If the current capacity is
         *              less than or equal to this value, reallocation occurs.
         * @param _increment The size by which the capacity is increased when reallocation
         *                   occurs.
         * @param _max_increment (optional) The maximum allowable increment to the capacity
         *                           during reallocation. Defaults to the largest possible size value.
         */
        HOT constexpr void allocate(size_t _hash, size_t _increment, size_t _max_increment = std::numeric_limits<size_t>::max()) {
            if (capacity() <= _hash) {
                reserve(utils::min(capacity() + _increment, _max_increment));
            }
        }

        /**
         * @brief Add a hash to the set.
         * @param _hash The hash value to add.
         */
        HOT constexpr void push(size_t _hash) {
            enable(_hash);
        }

        /**
         * @brief Adds a hash to the set using forward-construction semantics.
         * @details This function ensures that the hash is enabled within the set,
         *          forwarding the provided value and validating its type at compile-time.
         * @tparam T The type of the input value, which must be an integral type.
         * @param _hash The hash value to add.
         */
        template <typename T>
        HOT constexpr void emplace(T&& _hash) {

            static_assert(std::is_integral_v<std::decay_t<T>>, "Hash must be an integral type");

            enable(static_cast<size_t>(std::forward<T>(_hash)));
        }

        /**
         * @brief Remove a hash from the set.
         * @param _hash The hash value to remove.
         * @note This function does not resize the set.
         *
         * @see existence_set::clear()
         * @see existence_set::prune()
         */
        [[maybe_unused]] constexpr void erase(size_t _hash) noexcept {
            disable(_hash);
        }

        /**
         * @brief Check if the given hash exists in the set.
         * @param _hash The hash value to check.
         * @return True if the hash exists in the set, false otherwise.
         */
        [[maybe_unused, nodiscard]] HOT constexpr bool contains(size_t _hash) const noexcept {
            return _hash < size() && static_cast<bool>(c[_hash]);
        }

        /**
         * @brief Trim the set by removing trailing false values.
         *
         * @note Calling this function also trims the internal container of the set.
         *
         * @see existence_set::clear()
         * @see existence_set::erase()
         * @see existence_set::shrink_to_fit()
         */
        [[maybe_unused]] constexpr void trim() {

            auto it = c.rbegin();
            while (it != c.rend() && !static_cast<boolean_t>(*it)) {
                ++it;
            }
            c.erase(it.base(), c.end());
            c.shrink_to_fit();
        }

        /**
         * @brief Reserves memory for the set.
         *
         * @param _new_capacity The new capacity to reserve.
         * @note This method does not resize the set, only reserves memory for future elements.
         *
         * @see existence_set::capacity()
         * @see existence_set::resize()
         */
        [[maybe_unused]] HOT constexpr void reserve(size_t _new_capacity) { c.reserve(_new_capacity); }

        /**
         * @brief Resize the existence_set.
         *
         * This method resizes the existence_set by calling the resize method on the internal vector.
         *
         * @param _new_size The new size.
         * @param _new_value (optional) The value to fill new elements with. (default is false).
         * @note If the new size is smaller than the current size, the elements at the end will be removed.
         *       If the new size is greater than the current size, the new elements will be filled with the specified value.
         *
         * @see existence_set::clear()
         * @see existence_set::prune()
         * @see existence_set::reserve()
         */
        [[maybe_unused]] constexpr void resize(size_t _new_size, boolean_t _new_value = { false }) {
            c.resize(_new_size, _new_value);
        }

        /**
         * @brief Clears the content of the set.
         *
         *
         * @details This function clears all the elements in the existence set,
         *          effectively making it empty. After calling this function,
         *          the size of the set becomes zero.
         *
         * @note This does not change the capacity of the internal storage,
         *       it only clears the contents.
         *
         * @see existence_set::capacity()
         * @see existence_set::reserve()
         * @see existence_set::resize()
         * @see existence_set::shrink_to_fit()
         * @see existence_set::size()
         * @see existence_set::trim()
         */
        [[maybe_unused]] constexpr void clear() noexcept { c.clear(); }

        /**
         * @brief Trims the internal container of the set.
         * @details Removes trailing unused elements from the internal container of the existence set.
         *
         * @see existence_set::trim()
         */
        [[maybe_unused]] constexpr void shrink_to_fit() noexcept {
            try {
                c.shrink_to_fit();
            }
            catch (...) {} // NOLINT(*-empty-catch)
        }

        /**
         * @brief Get the size of the set.
         * @details Returns the number of elements in the set.
         *
         * @return The size of the set.
         */
        [[maybe_unused, nodiscard]] constexpr auto size() const noexcept { return c.size(); }

        /**
         * @brief Get the capacity of the set.
         * @details Returns the maximum number of elements that the set can hold.
         *
         * @return The capacity of the set.
         */
        [[maybe_unused, nodiscard]] constexpr auto capacity() const noexcept { return c.capacity(); }

        using iterator_t               = typename std::vector<layout_t>::              iterator;
        using const_iterator_t         = typename std::vector<layout_t>::        const_iterator;
        using reverse_iterator_t       = typename std::vector<layout_t>::      reverse_iterator;
        using const_reverse_iterator_t = typename std::vector<layout_t>::const_reverse_iterator;

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

#endif //CHDR_EXISTENCE_SET_HPP