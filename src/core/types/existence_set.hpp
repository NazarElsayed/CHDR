/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_EXISTENCE_SET_HPP
#define CHDR_EXISTENCE_SET_HPP

#include <vector>
#include <algorithm>

namespace chdr {

    struct lowest_memory_usage {}; /** @brief Each item is represented by 1 bit in memory. */
    struct    low_memory_usage {}; /** @brief Each item is represented by 8 bits in memory. */
    struct            balanced {}; /** @brief Each item is represented by 32 bits in memory. */
    struct highest_performance {}; /** @brief Each item is represented by the system size of a DWORD in memory. */

    template<typename>
    struct alignment {};
    template<> struct alignment<lowest_memory_usage> { using type [[maybe_unused]] =      bool; };
    template<> struct alignment<low_memory_usage>    { using type [[maybe_unused]] =      char; };
    template<> struct alignment<balanced>            { using type [[maybe_unused]] =  uint32_t; };
    template<> struct alignment<highest_performance> { using type [[maybe_unused]] = uintptr_t; };

    /**
     * @class DenseExistenceSet
     * @brief A set allowing for efficient existence checks without needing to store the original data in memory.
     *
     * @tparam alignment_type The alignment type used by the set.
     *
     * @see LowestMemoryUsage
     * @see LowMemoryUsage
     * @see Balanced
     * @see HighestPerformance
     */
    template <typename alignment_type = lowest_memory_usage>
    class existence_set {

        static_assert(
                std::is_same_v<alignment_type, lowest_memory_usage> ||
                std::is_same_v<alignment_type, low_memory_usage>    ||
                std::is_same_v<alignment_type, balanced>            ||
                std::is_same_v<alignment_type, highest_performance>,
            "AlignmentType must be one of the following: "
            "LowestMemoryUsage, LowMemoryUsage, Balanced, HighestPerformance"
        );

    private:

        using boolean_t = typename alignment<alignment_type>::Type;

        std::vector<boolean_t> m_bits;

    public:

        /**
         * @brief Initialise set.
         * @param[in] _capacity Initial capacity of the set. Must be larger than 0.
         */
        [[maybe_unused]] constexpr explicit existence_set(const size_t& _capacity = 1U) {
            reserve(_capacity);
        }

        /**
         * @brief Initialise set using a collection of items.
         * @details Please note: Duplicate entries will be merged.
         *
         * @param[in] _items Items to construct the set using.
         * @param[in] _capacity Initial capacity of the set. If a value less than 1 is assigned, it will use the size of the provided collection.
         */
        [[maybe_unused]] constexpr existence_set(const std::initializer_list<size_t>& _items, const size_t& _capacity = 0U) {

            size_t auto_capacity = _capacity;

            if (auto_capacity < 1U) {
                auto_capacity = std::max<size_t>(_items.size(), 1U);
            }

            reserve(auto_capacity);

            for (const auto& item : _items) {
                add(item);
            }
        }

        /**
         * @brief add a hash to the set.
         * @param[in] _hash The hash value to be added.
         */
        void add(const size_t& _hash) {

            if (_hash >= m_bits.size()) {
                resize(_hash + 1U);
            }

            m_bits[_hash] = static_cast<boolean_t>(true);
        }

        /**
         * @brief remove a hash from the set.
         * @param[in] _hash The hash value to be removed.
         * @note Please note that this function does not resize the set.
         *
         * @see set::prune()
         * @see set::clear()
         */
        [[maybe_unused]] void remove(const size_t& _hash) {

            if (_hash < m_bits.size()) {
                m_bits[_hash] = static_cast<boolean_t>(false);
            }
        }

        /**
         * @brief Check if the given hash exists in the set.
         * @param[in] _hash The hash value to check.
         * @return True if the hash exists in the set, false otherwise.
         */
        [[nodiscard]] bool contains(const size_t& _hash) const {
            return _hash < m_bits.size() && static_cast<bool>(m_bits[_hash]);
        }

        /**
         * @brief prune the set by removing trailing false values.
         *
         * @see set::clear()
         */
        [[maybe_unused]] void prune() {

            const auto it = std::find_if(
                    m_bits.rbegin(),
                    m_bits.rend(),
                    [](const auto& bit) constexpr {
                    return static_cast<bool>(bit);
                }
            );

            if (it != m_bits.rend()) {
                resize(std::distance(it, m_bits.rend()));
            }
            else {
                clear();
            }
        }

        /**
         * @brief Reserves memory for the DenseExistenceSet.
         *
         * @param _new_capacity The new capacity to reserve.
         * @note This method does not resize the set, only reserves memory for future elements.
         *
         * @see DenseExistenceSet::capacity()
         */
        [[maybe_unused]] void reserve(const size_t& _new_capacity) {
            m_bits.reserve(_new_capacity);
        }

        /**
         * @brief resize the DenseExistenceSet.
         *
         * This method resizes the DenseExistenceSet by calling the resize method on the internal vector.
         *
         * @param _new_size The new size to resize the set to.
         * @param _new_value The optional value to fill the new elements with. (default is false).
         * @note If the new size is smaller than the current size, the elements at the end will be removed.
         * If the new size is greater than the current size, the new elements will be filled with the specified value.
         *
         * @see DenseExistenceSet::reserve(const size_t&)
         * @see DenseExistenceSet::prune()
         * @see DenseExistenceSet::clear()
         */
        [[maybe_unused]] void resize(const size_t& _new_size, const boolean_t& _new_value = static_cast<boolean_t>(false)) {
            m_bits.resize(_new_size, _new_value);
        }

        /**
         * @brief clear the content of the set.
         * @details remove all elements from the set.
         */
        [[maybe_unused]] void clear() {
            m_bits.clear();
        }

        /**
         * @brief Trims unused elements from the end of the set.
         * @details Shrinks the internal container of the set to reduce the structure's overall memory footprint.
         */
        [[maybe_unused]] void trim() {
            m_bits.shrink_to_fit();
        }

        /**
         * @brief Get the size of the set.
         * @details Returns the number of elements in the set.
         *
         * @return The size of the set.
         */
        [[maybe_unused]] [[nodiscard]] constexpr auto size() const {
            return m_bits.size();
        }

        /**
         * @brief Get the capacity of the set.
         * @details Returns the maximum number of elements that the set can hold.
         *
         * @return The capacity of the set.
         */
        [[maybe_unused]] [[nodiscard]] constexpr auto capacity() const {
            return m_bits.capacity();
        }

        using               iterator_t = typename std::vector<alignment_type>::iterator;
        using         const_iterator_t = typename std::vector<alignment_type>::const_iterator;
        using       reverse_iterator_t = typename std::vector<alignment_type>::reverse_iterator;
        using const_reverse_iterator_t = typename std::vector<alignment_type>::const_reverse_iterator;

        [[maybe_unused]] [[nodiscard]]       iterator_t  begin()       { return m_bits.begin();  }
        [[maybe_unused]] [[nodiscard]] const_iterator_t  begin() const { return m_bits.begin();  }
        [[maybe_unused]] [[nodiscard]] const_iterator_t cbegin() const { return m_bits.cbegin(); }

        [[maybe_unused]] [[nodiscard]]       iterator_t  end()       { return m_bits.end();  }
        [[maybe_unused]] [[nodiscard]] const_iterator_t  end() const { return m_bits.end();  }
        [[maybe_unused]] [[nodiscard]] const_iterator_t cend() const { return m_bits.cend(); }

        [[maybe_unused]] [[nodiscard]]       reverse_iterator_t  rbegin()       { return m_bits.rbegin();  }
        [[maybe_unused]] [[nodiscard]] const_reverse_iterator_t  rbegin() const { return m_bits.rbegin();  }
        [[maybe_unused]] [[nodiscard]] const_reverse_iterator_t crbegin() const { return m_bits.crbegin(); }

        [[maybe_unused]] [[nodiscard]]       reverse_iterator_t  rend()       { return m_bits.rend();  }
        [[maybe_unused]] [[nodiscard]] const_reverse_iterator_t  rend() const { return m_bits.rend();  }
        [[maybe_unused]] [[nodiscard]] const_reverse_iterator_t crend() const { return m_bits.crend(); }
    };

} // CHDR

#endif //CHDR_EXISTENCE_SET_HPP