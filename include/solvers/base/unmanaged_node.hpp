/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_UNMANAGEDNODE_HPP
#define CHDR_UNMANAGEDNODE_HPP

#include "bnode.hpp"

// ReSharper disable once CppUnusedIncludeDirective
#include "../../utils/intrinsics.hpp" // NOLINT(*-include-cleaner)

namespace chdr::solvers {

    template<typename index_t>
    struct unmanaged_node : bnode<index_t> {

        const unmanaged_node* RESTRICT m_parent;

        /**
         * @brief Constructs an uninitialized node.
         *
         * This constructor creates a node with uninitialized members.
         */
        // ReSharper disable once CppPossiblyUninitializedMember
        constexpr unmanaged_node() noexcept {} // NOLINT(*-pro-type-member-init, *-use-equals-default)

        [[nodiscard]] constexpr unmanaged_node(const index_t& _index, const unmanaged_node* RESTRICT const _parent = nullptr) noexcept : bnode<index_t>(_index),
            m_parent(std::move(_parent)) {}
    };
}

#endif //CHDR_UNMANAGEDNODE_HPP