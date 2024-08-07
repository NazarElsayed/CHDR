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

        std::vector<typename Alignment<AlignmentType>::Type> m_Bits;

    public:

        void Add(const size_t& _hash) {

            if (_hash >= m_Bits.size()) {
                m_Bits.resize(_hash + 1U);
            }

            m_Bits[_hash] = static_cast<typename Alignment<AlignmentType>::Type>(true);
        }

        void Remove(const size_t& _hash) {

            if (_hash < m_Bits.size()) {
                m_Bits[_hash] = static_cast<typename Alignment<AlignmentType>::Type>(false);
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

        void Clear() { m_Bits.clear(); }

        constexpr auto Size() const { return m_Bits.size(); }
    };

} // CHDR

#endif //CHDR_DENSE_EXISTENCE_SET_HPP