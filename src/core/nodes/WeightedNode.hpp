/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_WEIGHTEDNODE_HPP
#define CHDR_WEIGHTEDNODE_HPP

#include <type_traits>

#include "nodes/base/INode.hpp"

namespace CHDR {

    template <typename W = bool>
    class WeightedNode {

        static_assert(std::is_integral_v<W>, "Type W must be an integral type.");

    private:

        W m_Value;

    public:

        constexpr WeightedNode(const W& _value = 0) : m_Value(_value) {}

        [[nodiscard]] bool IsActive() const {
            return m_Value != std::numeric_limits<W>::max();
        }

        [[nodiscard]] constexpr const W& Value() const {
            return m_Value;
        }

        constexpr void Value(const W& _value) {
            m_Value = _value;
        }
    };

} // CHDR

#endif //CHDR_WEIGHTEDNODE_HPP