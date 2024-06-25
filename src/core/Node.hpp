#ifndef CHDR_NODE_HPP
#define CHDR_NODE_HPP

#include <type_traits>

namespace CHDR {

    template <typename W = bool>
    class Node {

        static_assert(std::is_integral<W>::value, "Type W must be an integral type.");

    private:

        W m_Value;

    public:

        [[nodiscard]] constexpr bool IsActive() const {
            return m_Value != 0;
        }

        constexpr Node(const W& _value = 0) {
            m_Value = _value;
        }

        [[nodiscard]] constexpr W Value() const {
            return m_Value;
        }

        constexpr void Value(const W& value) {
            m_Value = value;
        }
    };

} // CHDR

#endif //CHDR_NODE_HPP