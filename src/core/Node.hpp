#ifndef CHDR_NODE_HPP
#define CHDR_NODE_HPP

#include <type_traits>

namespace CHDR {

    template <typename W = bool>
    struct Node {

        static_assert(std::is_integral<W>::value, "Type W must be an integral type.");

        W m_Value;

        constexpr Node(const W& _value) {
            m_Value = _value;
        }

        [[nodiscard]] constexpr bool IsActive() const {
            return m_Value != 0;
        }
    };

} // CHDR

#endif //CHDR_NODE_HPP