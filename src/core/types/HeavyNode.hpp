#ifndef CHDR_HEAVYNODE_HPP
#define CHDR_HEAVYNODE_HPP

#include <climits>
#include <type_traits>

#include "NodeData.hpp"

namespace CHDR {

    template <typename W = bool>
    class HeavyNode {

        static_assert(std::is_integral_v<W>, "Type W must be an integral type.");

    private:

        W m_Value;
        NodeData* m_Data;

    public:

        [[nodiscard]] constexpr bool IsActive() const {
            return m_Value != std::numeric_limits<W>::max();
        }

        constexpr explicit HeavyNode(const W& _value = 0, NodeData* _data = nullptr) :
            m_Value(_value),
            m_Data(_data) {}

        ~HeavyNode() {
            delete m_Data;
        }

        [[nodiscard]] constexpr W Value() const {
            return m_Value;
        }

        constexpr void Value(const W& _value) {
            m_Value = _value;
        }

        constexpr auto& Data() {
            if (m_Data == nullptr) {
                m_Data = new NodeData(0, INT_MAX, 0, 0);
            }

            return m_Data;
        }

        constexpr void Data(NodeData& _data) {
            m_Data = &_data;
        }
    };
} // CHDR

#endif //CHDR_HEAVYNODE_HPP
