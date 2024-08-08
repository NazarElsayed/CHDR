#ifndef CHDR_HEAVYNODE_HPP
#define CHDR_HEAVYNODE_HPP

#include <type_traits>

#include "NodeData.hpp"

namespace CHDR {

    template <typename Ts, typename W = bool>
    class HeavyNode {

        static_assert(std::is_integral_v<W>, "Type W must be an integral type.");

    private:

        W m_Value;
        NodeData<Ts> m_Data;

    public:

        [[nodiscard]] constexpr bool IsActive() const {
            return m_Value != std::numeric_limits<W>::max();
        }

        constexpr HeavyNode(NodeData<Ts> _data, const W& _value = 0) :
            m_Value(_value),
            m_Data(_data) {}

        [[nodiscard]] constexpr W Value() const {
            return m_Value;
        }

        constexpr void Value(const W& _value) {
            m_Value = _value;
        }

        [[nodiscard]] constexpr auto& Data() const {
            return m_Data;
        }

        constexpr void Data(const NodeData<Ts>& _data) {
            m_Data = _data;
        }
    };
} // CHDR

#endif //CHDR_HEAVYNODE_HPP
