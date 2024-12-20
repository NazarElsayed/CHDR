/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_WEIGHTED_NODE_HPP
#define CHDR_WEIGHTED_NODE_HPP

#include <type_traits>

namespace chdr::mazes {

    template <typename W = bool>
    class weighted_node {

        static_assert(std::is_integral_v<W>, "Type W must be an integral type.");
        static_assert(std::numeric_limits<W>::is_specialized, "W must be a numeric type with defined numeric limits.");

    private:

        W m_value;

    public:

        constexpr weighted_node(const W& _value = 0) noexcept : m_value(_value) {}

        [[nodiscard]] constexpr bool is_active() const noexcept {
            return m_value != std::numeric_limits<W>::max();
        }

        [[nodiscard]] constexpr const W& value() const noexcept {
            return m_value;
        }

        constexpr void value(const W& _value) noexcept {
            m_value = _value;
        }
    };

} //chdr::mazes

#endif //CHDR_WEIGHTED_NODE_HPP