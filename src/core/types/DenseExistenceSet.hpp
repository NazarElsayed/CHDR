#ifndef CHDR_PERFECT_SCALABLE_BLOOM_FILTER_HPP
#define CHDR_PERFECT_SCALABLE_BLOOM_FILTER_HPP

#include <vector>

namespace CHDR {

    struct LowestMemory       {};
    struct LowMemory          {};
    struct Balanced           {};
    struct HighestPerformance {};

    template<typename T>
    struct Alignment {};

    template<>
    struct Alignment<LowestMemory> {
        using Type = bool;
    };

    template<>
    struct Alignment<LowMemory> {
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

    template <typename AlignmentType = LowestMemory>
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

            if (_hash >= m_Bits.size()) {
                m_Bits.resize(_hash + 1U);

            }

            m_Bits[_hash] = static_cast<typename Alignment<AlignmentType>::Type>(false);
        }

        bool Contains(const size_t& _hash) {

            if (_hash >= m_Bits.size()) {
                m_Bits.resize(_hash + 1U);
            }

            return static_cast<bool>(m_Bits[_hash]);
        }

        void Clear() {
            m_Bits.clear();
        }
    };

} // CHDR

#endif //CHDR_PERFECT_SCALABLE_BLOOM_FILTER_HPP