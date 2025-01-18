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

    template <typename weight_t>
    struct weighted_node {

        static_assert(std::is_integral_v<weight_t>, "Type W must be an integral type.");
        static_assert(std::numeric_limits<weight_t>::is_specialized, "W must be a numeric type with defined numeric limits.");

    private:

        weight_t m_value;

    public:

        [[nodiscard]] constexpr weighted_node() = delete;
        ~weighted_node() noexcept = default;

        [[nodiscard]] constexpr weighted_node(const weight_t& _value) noexcept : m_value(_value) {}

        [[nodiscard]] constexpr weighted_node           (const weighted_node&) noexcept = default;
                      constexpr weighted_node& operator=(const weighted_node&) noexcept = default;

        [[nodiscard]] constexpr weighted_node(weighted_node&&) noexcept = default;

#if __cplusplus > 202302L
        constexpr
#endif
        weighted_node& operator=(weighted_node&&) noexcept = default;
        
        [[nodiscard]] constexpr bool is_active() const noexcept {
            return m_value != std::numeric_limits<weight_t>::max();
        }

        [[nodiscard]] constexpr const weight_t& value() const noexcept { return m_value; }
        constexpr void value(const weight_t& _value) noexcept { m_value = _value; }
    };

} //chdr::mazes

#endif //CHDR_WEIGHTED_NODE_HPP