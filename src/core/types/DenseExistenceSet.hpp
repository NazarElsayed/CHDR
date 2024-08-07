#ifndef CHDR_DENSE_EXISTENCE_SET_HPP
#define CHDR_DENSE_EXISTENCE_SET_HPP

#include <vector>

namespace CHDR {

    struct LowestMemoryUsage  {};
    struct LowMemoryUsage     {};
    struct Balanced           {};
    struct HighestPerformance {};

    template<typename T>
    struct Alignment {};

    template<>
    struct Alignment<LowestMemoryUsage> {
        using Type = bool;
    };

    template<>
    struct Alignment<LowMemoryUsage> {
        using Type = char;
    };

    template<>
    struct Alignment<Balanced> {
        using Type = int32_t;
    };

    template<>
    struct Alignment<HighestPerformance> {
        using Type = intptr_t;
    };

    template <typename AlignmentType = LowestMemoryUsage>
    class DenseExistenceSet {

    private:

        using boolean_t = typename Alignment<AlignmentType>::Type;

        std::vector<boolean_t> m_Bits;

    public:

        /**
         * @brief Initialise DenseExistenceSet.
         * @param[in] _capacity Initial capacity of the set. Must be larger than 0.
         */
        constexpr DenseExistenceSet(const size_t& _capacity = 1U) {
            m_Bits.resize(_capacity);
        }

        /**
         * @brief Initialise DenseExistenceSet using a collection of items.
         * @details Please note: Duplicate entries will be merged.
         *
         * @param[in] _items Items to construct the set using.
         * @param[in] _capacity Initial capacity of the set. If a value less than 1 is assigned, it will use the size of the provided collection.
         */
        constexpr DenseExistenceSet(const std::initializer_list<size_t>& _items, const size_t& _capacity = 0U) {

            size_t auto_capacity = _capacity;

            if (auto_capacity < 1U) {
                auto_capacity = std::max<size_t>(_items.size(), 1U);
            }

            m_Bits.reserve(auto_capacity);

            for (const auto& item : _items) {
                Add(item);
            }
        }

        void Add(const size_t& _hash) {

            if (_hash >= m_Bits.size()) {
                m_Bits.resize(_hash + 1U);
            }

            m_Bits[_hash] = static_cast<boolean_t>(true);
        }

        void Remove(const size_t& _hash) {

            if (_hash < m_Bits.size()) {
                m_Bits[_hash] = static_cast<boolean_t>(false);
            }
        }

        [[nodiscard]] inline bool Contains(const size_t& _hash) const {
            return _hash < m_Bits.size() && static_cast<bool>(m_Bits[_hash]);
        }

        void Trim() {

            const auto it = std::find_if(
                m_Bits.rbegin(),
                m_Bits.rend(),
                [](const auto& bit) constexpr {
                    return static_cast<bool>(bit);
                }
            );

            if (it != m_Bits.rend()) {
                m_Bits.resize(std::distance(it, m_Bits.rend()));
            }
            else {
                Clear();
            }
        }

        void Clear() {
            m_Bits.clear();
        }

        constexpr auto Size() const {
            return m_Bits.size();
        }
    };

} // CHDR

#endif //CHDR_DENSE_EXISTENCE_SET_HPP