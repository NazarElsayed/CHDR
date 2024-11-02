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