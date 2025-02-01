/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_BNODE_HPP
#define CHDR_BNODE_HPP

/**
  * @file bnode.hpp
 */

namespace chdr::solvers {

    /**
     * @nosubgrouping
     * @struct bnode
     * @brief Base class for pathfinding nodes.
     * @details Represents the fundamental functionality of a node in a search context,
     *          storing an index representing an identifier.
     *          It can be inherited to extend its functionality.
     * @remarks The stored identifier does not need to be unique.
     * @tparam index_t The type of the identifier. Must be an integral type.
     *
     * @see unmanaged_node
     * @see managed_node
     */
    template<typename index_t>
    struct bnode {

        /**
         * @brief The index or identifier of the node.
         *
         * This member represents the identifier for the node within the pathfinding
         * or search algorithm context. The identifier is typically used to
         * distinguish or reference specific nodes.
         *
         * @details The type of the index is templated and must be an integral type.
         *          The value stored in this member does not require uniqueness,
         *          and its interpretation depends on the specific application or derived class.
         */
        index_t m_index;

        /**
         * @name Constructors
         * @{
         */

        /**
         * @brief Constructs an uninitialized BNode.
         *
         * This constructor creates a bnode with uninitialized members.
         */
        // ReSharper disable once CppPossiblyUninitializedMember
        constexpr bnode() noexcept {} // NOLINT(*-pro-type-member-init, *-use-equals-default)
        ~bnode() noexcept = default;

        HOT constexpr bnode(index_t _index) noexcept : m_index(_index) {}

        constexpr bnode           (const bnode&) noexcept = delete;
        constexpr bnode& operator=(const bnode&) noexcept = delete;

        [[nodiscard]] HOT constexpr bnode(bnode&&) noexcept = default;

#if __cplusplus > 202302L
        constexpr
#endif
        bnode& operator=(bnode&&) noexcept = default;

        /**
         * @}
         */

        [[nodiscard]] friend constexpr bool operator == (const bnode& _a, const bnode& _b) noexcept { return _a.m_index == _b.m_index; }

    };

} //chdr::solvers

#endif //CHDR_BNODE_HPP