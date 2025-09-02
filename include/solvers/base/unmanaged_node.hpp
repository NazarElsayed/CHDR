/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_UNMANAGEDNODE_HPP
#define CHDR_UNMANAGEDNODE_HPP

/**
 * @file unmanaged_node.hpp
 */

#include "bnode.hpp"

// ReSharper disable once CppUnusedIncludeDirective
#include "../../utils/intrinsics.hpp" // NOLINT(*-include-cleaner)

namespace chdr::solvers {

    /**
     * @struct unmanaged_node
     * @brief A hierarchical node in a pathfinding context, requires manual memory management.
     *
     * @details This structure includes a pointer to its parent, allowing it to handle
     *          the hierarchical relationship between nodes.
     *
     * @warning unmanaged_node does not manage the lifetime of its parent.
     *          Careful considerations should be made to ensure it is cleaned up correctly.
     *
     * @tparam index_t The type representing the identifier, derived from bnode.
     *                 It must be an integral type.
     *
     * @see bnode
     */
    template<typename index_t>
    struct unmanaged_node : bnode<index_t> {

        /**
         * @brief A pointer to the parent node in the hierarchical structure.
         *
         * This member allows traversal of the hierarchy by referencing the immediate
         * parent of the current node. Use of `restrict` indicates that this pointer
         * is not aliased, improving optimisation in certain compilers.
         *
         * @warning The lifetime of the parent node is not managed by this member. Users must
         *          ensure that the parent node remains valid for the duration of any access
         *          via this pointer.
         *
         * @remark If `m_parent` is `nullptr`, it indicates that the node is a root node
         *         and does not have a parent.
         */
        const unmanaged_node* RESTRICT m_parent;

        /**
         * @name Constructors
         * @{
         */

        /**
         * @brief Constructs an uninitialized node.
         *
         * This constructor creates a node with uninitialized members.
         */
        // ReSharper disable once CppPossiblyUninitializedMember
        [[nodiscard]] constexpr unmanaged_node() noexcept {} // NOLINT(*-pro-type-member-init, *-use-equals-default)

        ~unmanaged_node() noexcept = default;

        constexpr unmanaged_node           (const unmanaged_node&) = default;
        constexpr unmanaged_node& operator=(const unmanaged_node&) = default;

        [[nodiscard]] HOT constexpr unmanaged_node(unmanaged_node&&) noexcept = default;

#if __cplusplus > 202302L
        constexpr
#endif
        unmanaged_node& operator=(unmanaged_node&&) noexcept = default;

        /**
         * @brief Constructs a node with a specified index and an optional parent pointer.
         *
         * @details This constructor initialises a node, allowing both the index and parent node to be specified.
         *          The parent pointer is useful in defining hierarchical relationships between nodes in tree-like
         *          structures.
         *          By default, the parent node is set to `nullptr`, which means the node is a root in the hierarchy.
         *
         * @param _index The index or identifier of the node, used in the context of pathfinding or search.
         * @param [in] _parent (optional) A pointer to the parent node in the hierarchy. If `nullptr`,
         *                                the node will be considered a root node.
         *
         * @warning Memory associated with the parent node is not managed by this constructor, or the class itself.
         *          It is the responsibility of the caller to ensure the parent node is valid during the lifetime
         *          of this object.
         *
         * @warning If `_parent` is not `nullptr`, it must point to a valid and initialised
         *          unmanaged_node. Failing to ensure this may lead to undefined behaviour.
         */
        [[nodiscard]] HOT constexpr unmanaged_node(index_t _index, const unmanaged_node* RESTRICT const _parent = nullptr) noexcept : bnode<index_t>(_index),
            m_parent(_parent) {}

        /**
         * @}
         */
    };

}

#endif //CHDR_UNMANAGEDNODE_HPP