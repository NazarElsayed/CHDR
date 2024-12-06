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
    struct bNode {

        index_t m_index;

        /**
         * @brief Constructs an uninitialized BNode.
         *
         * This constructor creates an BNode with uninitialized members.
         */
        constexpr bNode() {} // NOLINT(*-pro-type-member-init, *-use-equals-default)

        bNode(const index_t& _index) : m_index(_index) {};

        [[nodiscard]] constexpr bool operator == (const bNode& _node) const { return m_index == _node.m_index; }

    };
}

#endif //CHDR_BNODE_HPP