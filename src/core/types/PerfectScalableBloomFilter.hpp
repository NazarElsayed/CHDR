#ifndef CHDR_PERFECT_SCALABLE_BLOOM_FILTER_HPP
#define CHDR_PERFECT_SCALABLE_BLOOM_FILTER_HPP

#include <vector>

namespace CHDR {

    class PerfectScalableBloomFilter {

    private:

        std::vector<bool> m_Bits;

    public:

        void Add(const size_t& _hash) {

            if (_hash >= m_Bits.size()) {
                m_Bits.resize(_hash + 1U);
            }

            m_Bits[_hash] = true;
        }

        bool Contains(const size_t& _hash) {

            if (_hash >= m_Bits.size()) {
                m_Bits.resize(_hash + 1U);
            }

            return m_Bits[_hash];
        }

        void Clear() {
            m_Bits.clear();
        }
    };

} // CHDR

#endif //CHDR_PERFECT_SCALABLE_BLOOM_FILTER_HPP