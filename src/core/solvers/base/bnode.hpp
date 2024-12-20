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

        constexpr bnode(const index_t& _index) noexcept : m_index(_index) {}

        [[nodiscard]] friend constexpr bool operator == (const bnode& _a, const bnode& _b) noexcept { return _a.m_index == _b.m_index; }

    };

} //chdr::solvers

#endif //CHDR_BNODE_HPP