/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_IDNODE_HPP
#define CHDR_IDNODE_HPP

/**
 * @file id_node.hpp
 */

#include <type_traits>

namespace chdr::mazes {

    /**
     * @nosubgrouping
     * @class id_node
     * @brief Represents a node with customisable identifier.
     * @details Encapsulates an identifier of a user-defined integral type.
     * @note Is equivalent to its stored identifier and can be reinterpreted as the identifier's type.
     * @remarks The stored identifier does not need to be unique.
     * @tparam index_t Identifier type. Must be integral type.
     */
    template <typename index_t>
    class id_node final {

        static_assert(std::is_integral_v<index_t>, "Type index_t must be an integral type.");

    private:

        index_t m_id;

    public:

        /**
         * @name Constructors
         * @{
         */

        /**
         * @brief Constructs an instance using the specified identifier.
         * @param _id The identifier to initialise with.
         */
        [[nodiscard]] constexpr id_node(index_t _id) noexcept : m_id(_id) {}

        ~id_node() noexcept = default;

        [[nodiscard]] id_node           (const id_node&) = delete;
                      id_node& operator=(const id_node&) = delete;

        [[nodiscard]] constexpr id_node(id_node&&) noexcept = default;

        /**
         * @}
         */

#if __cplusplus > 202302L
        constexpr
#endif
        id_node& operator=(id_node&&) noexcept = default;

        /**
         * @brief Whether the node is active or not.
         * @warning For id_node instances, this always returns true.
         * @return true
         */
        [[maybe_unused, nodiscard]] constexpr static bool is_active() noexcept { return true; }

        /**
         * @brief Retrieves the identifier associated with the node.
         * @return The identifier stored in the node.
         */
        [[maybe_unused, nodiscard]] constexpr index_t id() const noexcept { return m_id; }

        /**
         * @brief Assigns a new identifier to the node.
         * @param _id The new identifier to assign to the node.
         */
        [[maybe_unused]] constexpr void id(index_t _id) noexcept { m_id = _id; }
    };

} //chdr::mazes

#endif //CHDR_IDNODE_HPP