/*
* Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_BNODE_HPP
#define CHDR_BNODE_HPP

namespace chdr::solvers {

    template<typename index_t>
    struct bnode {

        index_t m_index;

        /**
         * @brief Constructs an uninitialized BNode.
         *
         * This constructor creates an BNode with uninitialized members.
         */
        // ReSharper disable once CppPossiblyUninitializedMember
        constexpr bnode() noexcept {} // NOLINT(*-pro-type-member-init, *-use-equals-default)

        bnode(const index_t& _index) noexcept : m_index(_index) {}

        [[nodiscard]] constexpr bool operator == (const bnode& _node) const noexcept { return m_index == _node.m_index; }

    };

} //chdr::solvers

#endif //CHDR_BNODE_HPP