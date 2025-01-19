/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_UNMANAGEDNODE_HPP
#define CHDR_UNMANAGEDNODE_HPP

#include <memory>

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
        [[nodiscard]] constexpr unmanaged_node() noexcept {} // NOLINT(*-pro-type-member-init, *-use-equals-default)
        ~unmanaged_node() noexcept = default;

        constexpr unmanaged_node           (const unmanaged_node&) = delete;
        constexpr unmanaged_node& operator=(const unmanaged_node&) = delete;

        [[nodiscard]] constexpr unmanaged_node(unmanaged_node&&) noexcept = default;

#if __cplusplus > 202302L
        constexpr
#endif
        unmanaged_node& operator=(unmanaged_node&&) noexcept = default;

        [[nodiscard]] constexpr unmanaged_node(const index_t& _index, const unmanaged_node* RESTRICT const _parent = nullptr) noexcept : bnode<index_t>(_index),
            m_parent(std::move(_parent)) {}
    };
}

#endif //CHDR_UNMANAGEDNODE_HPP