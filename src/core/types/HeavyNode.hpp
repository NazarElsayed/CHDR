#ifndef CHDR_HEAVYNODE_HPP
#define CHDR_HEAVYNODE_HPP

#include <limits.h>
#include <type_traits>

#include "NodeData.hpp"

namespace CHDR {

    template <typename W = bool>
    class HeavyNode {

        static_assert(std::is_integral_v<W>, "Type W must be an integral type.");

    private:

        W m_Value;
        NodeData m_Data;

    public:

        [[nodiscard]] constexpr bool IsActive() const {
            return m_Value != std::numeric_limits<W>::max();
        }

        constexpr HeavyNode(const W& _value = 0, NodeData _data = {0, INT_MAX, 0, 0}) :
            m_Value(_value),
            m_Data(_data) {}

        [[nodiscard]] constexpr W Value() const {
            return m_Value;
        }

        constexpr void Value(const W& _value) {
            m_Value = _value;
        }

        constexpr auto& Data() {
            return m_Data;
        }

        constexpr void Data(const NodeData& _data) {
            m_Data = _data;
        }
    };
} // CHDR

#endif //CHDR_HEAVYNODE_HPP
