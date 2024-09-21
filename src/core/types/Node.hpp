/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_NODE_HPP
#define CHDR_NODE_HPP

#include <type_traits>

namespace CHDR {

    template <typename W = bool>
    class Node {

        static_assert(std::is_integral_v<W>, "Type W must be an integral type.");

    private:

        W m_Value;

    public:

        [[nodiscard]] constexpr bool IsActive() const {
            return m_Value != std::numeric_limits<W>::max();
        }

        constexpr Node(const W& _value = 0) :
            m_Value(_value) {}

        [[nodiscard]] constexpr W Value() const {
            return m_Value;
        }

        constexpr void Value(const W& _value) {
            m_Value = _value;
        }
    };

} // CHDR

#endif //CHDR_NODE_HPP