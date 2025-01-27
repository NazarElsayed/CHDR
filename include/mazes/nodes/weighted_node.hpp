/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_WEIGHTED_NODE_HPP
#define CHDR_WEIGHTED_NODE_HPP

/**
 * @file weighted_node.hpp
 */

#include <limits>
#include <type_traits>

namespace chdr::mazes {

    /**
     * @nosubgrouping
     * @struct weighted_node
     * @brief Represents a node with a customisable weight.
     * @details The weighted_node class encapsulates a weight of a user-defined integral type.
     * @note The weighted_node class is equivalent to its stored weight and can be reinterpreted as the weights's type.
     * @remarks The stored weight may be signed or unsigned.
     * @warning For weighted_node instances, the maximum value of weight_t is reserved for inactive nodes.
     * @tparam weight_t The type representing weight. Must be an integral type.
     */
    template <typename weight_t>
    struct weighted_node {

        static_assert(std::is_integral_v<weight_t>, "Type W must be an integral type.");
        static_assert(std::numeric_limits<weight_t>::is_specialized, "W must be a numeric type with defined numeric limits.");

    private:

        weight_t m_value;

    public:

        /**
         * @name Constructors
         * @{
         */

        [[nodiscard]] constexpr weighted_node() = delete;

        /**
         * @brief Constructs a weighted_node instance with the specified weight.
         * @param _value The weight to initialise the weighted_node with.
         * @warning For weighted_node instances, the maximum value of weight_t is reserved for inactive nodes.
         * @return A weighted_node instance with the specified weight.
         */
        [[nodiscard]] constexpr weighted_node(weight_t _value) noexcept : m_value(_value) {}

        ~weighted_node() noexcept = default;

        [[nodiscard]] constexpr weighted_node           (const weighted_node&) noexcept = delete;
                      constexpr weighted_node& operator=(const weighted_node&) noexcept = delete;

        [[nodiscard]] constexpr weighted_node(weighted_node&&) noexcept = default;

        /**
         * @}
         */

#if __cplusplus > 202302L
        constexpr
#endif
        weighted_node& operator=(weighted_node&&) noexcept = default;

        /**
         * @brief Whether the node is active or not.
         * @warning For weighted_node instances, the maximum value of weight_t is reserved for inactive nodes.
         * @return If the weight equals the maximum possible weight, false. Otherwise, true.
         */
        [[nodiscard]] HOT constexpr bool is_active() const noexcept {
            return m_value != std::numeric_limits<weight_t>::max();
        }

        /**
         * @brief Retrieves the weight of the node.
         * @warning For weighted_node instances, the maximum value of weight_t is reserved for inactive nodes.
         * @return The weight of the node.
         */
        [[nodiscard]] constexpr weight_t value() const noexcept { return m_value; }

        /**
         * @brief Assigns a new weight to the node.
         * @warning For weighted_node instances, the maximum value of weight_t is reserved for inactive nodes.
         * @param _value The new weight to assign to the node.
         */
        constexpr void value(weight_t _value) noexcept { m_value = _value; }
    };

} //chdr::mazes

#endif //CHDR_WEIGHTED_NODE_HPP