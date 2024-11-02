/*
* Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_BNODE_HPP
#define CHDR_BNODE_HPP

#include "../../utils/Intrinsics.hpp"

#include <memory>

namespace CHDR::Solvers {

    template<typename index_t>
    struct BNode {

        index_t m_Index;

        /**
         * @brief Constructs an uninitialized BNode.
         *
         * This constructor creates an BNode with uninitialized members.
         */
        constexpr BNode() {} // NOLINT(*-pro-type-member-init, *-use-equals-default)

        BNode(const index_t& _index) : m_Index(_index) {};

        [[nodiscard]] constexpr bool operator == (const BNode& _node) const { return m_Index == _node.m_Index; }

    };
}

#endif //CHDR_BNODE_HPP