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

    /** @brief One bit represents each item in memory.*/
    struct lowest_memory_usage final {};

    /** @brief Eight bits represent each item in memory.*/
    struct high_performance final {};

    template<typename>
    struct alignment {};
    template<> struct alignment<lowest_memory_usage> final { using type_t [[maybe_unused]] = bool; };
    template<> struct alignment<high_performance>    final { using type_t [[maybe_unused]] = char; };

    /**
     * @nosubgrouping
     * @class existence_set
     * @details A set allowing for efficient existence checks without needing to store the original data in memory.
     *
     * @tparam alignment_type The alignment type used by the set.
     *
     * @see lowest_memory_usage
     * @see high_performance
     */
    template <typename alignment_type = high_performance>
    class existence_set {

        static_assert(
            std::is_same_v<alignment_type, lowest_memory_usage> ||
            std::is_same_v<alignment_type, high_performance>,
            "alignment_type must be one of the following: "
            "lowest_memory_usage, high_performance"
        );

    private:

        using boolean_t = typename alignment<alignment_type>::type_t;

        std::pmr::vector<boolean_t> c;

        constexpr void enable(const size_t& _hash) {

            if (_hash >= c.size()) {
                resize(_hash + 1U);
            }

#if defined(GCC)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstringop-overflow"
#endif

            c[_hash] = static_cast<boolean_t>(true);

#if defined(GCC)
#pragma GCC diagnostic pop
#endif
        }

        constexpr void disable(const size_t& _hash) noexcept {

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
         * @brief Default Constructor.
         * @details Initialises the set. A custom polymorphic memory resource can be optionally provided.
         * @param[in, out] _resource (optional) Custom memory resource.
         */
        [[maybe_unused]] constexpr existence_set(std::pmr::memory_resource* _resource = std::pmr::get_default_resource()) noexcept : c(_resource) {}

        /**
         * @brief Constructor with a predefined capacity.
         * @details Initialises the set with a predefined capacity. A custom polymorphic memory resource can be optionally provided.
         * @param[in] _capacity Initial capacity of the set. Must be larger than 0.
         * @param[in, out] _resource (optional) Custom memory resource.
         */
        [[maybe_unused]] constexpr explicit existence_set(const size_t& _capacity, std::pmr::memory_resource* _resource = std::pmr::get_default_resource()) : c(_resource) {
            reserve(_capacity);
        }

        /**
         * @brief Constructor with a collection of items.
         * @details Initialises the set with a predefined list of items. A custom polymorphic memory resource can be optionally provided.
         * @note Please note: Duplicate entries will be merged into single entries.
         *
         * @param[in] _items Items to construct the set using.
         * @param[in, out] _resource (optional) Custom memory resource.
         */
        [[maybe_unused]] constexpr existence_set(const std::initializer_list<size_t>& _items, std::pmr::memory_resource* _resource = std::pmr::get_default_resource()) {
            reserve(_items.size());
            for (const auto& item : _items) {
                push(item);
            }
        }

        /**
         * @}
         **/

        /**
         * @brief Preallocates memory based on the hash and bucket size.
         * @param[in] _hash The value for which memory needs to be preallocated.
         * @param[in] _increment Size of bucket to be considered for memory allocation.
         * @param[in] _max_increment (optional) Maximum limit for the memory allocation.
         */
        HOT constexpr void allocate(const size_t& _hash, const size_t& _increment, const size_t& _max_increment = std::numeric_limits<size_t>::max()) {
            if (capacity() <= _hash) {
                reserve(utils::min(capacity() + _increment, _max_increment));
            }
        }

        /**
         * @brief Add a hash to the set.
         * @param[in] _hash The hash value to be added.
         */
        HOT constexpr void push(const size_t& _hash) {
            enable(_hash);
        }

        /**
         * @brief Add a hash to the set.
         * @param[in] _hash The hash value to be added.
         */
        template <typename T>
        HOT constexpr void emplace(T&& _hash) {

            static_assert(std::is_integral_v<std::decay_t<T>>, "Hash must be an integral type");

            enable(static_cast<size_t>(std::forward<T>(_hash)));
        }

        /**
         * @brief remove a hash from the set.
         * @param[in] _hash The hash value to be removed.
         * @note Please note that this function does not resize the set.
         *
         * @see set::prune()
         * @see set::clear()
         */
        [[maybe_unused]] constexpr void erase(const size_t& _hash) noexcept {
            disable(_hash);
        }

        /**
         * @brief Check if the given hash exists in the set.
         * @param[in] _hash The hash value to check.
         * @return True if the hash exists in the set, false otherwise.
         */
        [[maybe_unused, nodiscard]] HOT constexpr bool contains(const size_t& _hash) const noexcept {
            return _hash < size() && static_cast<bool>(c[_hash]);
        }

        /**
         * @brief trim the set by removing trailing false values.
         *
         * @see set::clear()
         */
        [[maybe_unused]] constexpr void trim() {

            auto it = c.rbegin();
            while (it != c.rend() && !static_cast<boolean_t>(*it)) {
                ++it;
            }
            c.erase(it.base(), c.end());
        }

        /**
         * @brief Reserves memory for the existence_set.
         *
         * @param _newCapacity The new capacity to reserve.
         * @note This method does not resize the set, only reserves memory for future elements.
         *
         * @see existence_set::capacity()
         */
        [[maybe_unused]] HOT constexpr void reserve(const size_t& _newCapacity) { c.reserve(_newCapacity); }

        /**
         * @brief resize the existence_set.
         *
         * This method resizes the existence_set by calling the resize method on the internal vector.
         *
         * @param _newSize The new size to resize the set to.
         * @param _newValue The optional value to fill the new elements with. (default is false).
         * @note If the new size is smaller than the current size, the elements at the end will be removed.
         * If the new size is greater than the current size, the new elements will be filled with the specified value.
         *
         * @see existence_set::reserve(const size_t&)
         * @see existence_set::prune()
         * @see existence_set::clear()
         */
        [[maybe_unused]] constexpr void resize(const size_t& _newSize, const boolean_t& _newValue = static_cast<boolean_t>(false)) {
            c.resize(_newSize, _newValue);
        }

        /**
         * @brief clear the content of the set.
         * @details remove all elements from the set.
         */
        [[maybe_unused]] constexpr void clear() noexcept { c.clear(); }

        /**
         * @brief Trims unused elements from the end of the set.
         * @details Shrinks the internal container of the set to reduce the structure's overall memory footprint.
         */
        [[maybe_unused]] constexpr void shrink_to_fit() { c.shrink_to_fit(); }

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

        using               iterator_t = typename std::vector<alignment_type>::              iterator;
        using         const_iterator_t = typename std::vector<alignment_type>::        const_iterator;
        using       reverse_iterator_t = typename std::vector<alignment_type>::      reverse_iterator;
        using const_reverse_iterator_t = typename std::vector<alignment_type>::const_reverse_iterator;

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