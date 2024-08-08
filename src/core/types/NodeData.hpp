#ifndef CHDR_NODEDATA_HPP
#define CHDR_NODEDATA_HPP
#include "Heap.hpp"

namespace CHDR {

    template<typename Ts>
    struct NodeData : IHeapItem {

        Ts m_GScore;
        Ts m_FScore;

        size_t m_Parent;

        constexpr NodeData(const Ts& _gScore, const Ts& _hScore, const size_t _parent) : IHeapItem(),
            m_GScore(_gScore),
            m_FScore(_gScore + _hScore),
            m_Parent(_parent) {}
    };
}

#endif //CHDR_NODEDATA_HPP
