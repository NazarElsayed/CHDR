#ifndef CHDR_NODEDATA_HPP
#define CHDR_NODEDATA_HPP
#include "Heap.hpp"

namespace CHDR {

    struct NodeData : IHeapItem {

        size_t m_Position;

        size_t m_GScore;
        size_t m_FScore;

        size_t m_Parent;

        bool m_Closed;

        constexpr NodeData(const size_t _position, const size_t& _gScore, const size_t& _hScore, const size_t& _parent) : IHeapItem(),
            m_Position(_position),
            m_GScore(_gScore),
            m_FScore(_gScore + _hScore),
            m_Parent(_parent),
            m_Closed(false) {}
    };
}

#endif //CHDR_NODEDATA_HPP
