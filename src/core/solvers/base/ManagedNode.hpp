/*
* Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_MANAGEDNODE_HPP
#define CHDR_MANAGEDNODE_HPP

#include "../../utils/Intrinsics.hpp"
#include "BNode.hpp"

#include <memory>

namespace CHDR::Solvers {

    template<typename index_t>
    struct ManagedNode : public BNode<index_t> {

        std::shared_ptr<const ManagedNode<index_t>> m_Parent;

        /**
         * @brief Constructs an uninitialized ManagedNode.
         *
         * This constructor creates an ManagedNode with uninitialized members.
         */
        constexpr ManagedNode() {} // NOLINT(*-pro-type-member-init, *-use-equals-default)

        [[nodiscard]] constexpr ManagedNode(const index_t& _index) :
            BNode<index_t>(_index),
            m_Parent() {}

        [[nodiscard]] constexpr ManagedNode(const index_t& _index, ManagedNode<index_t>&& _parent) :
            BNode<index_t>(_index)
        {
            m_Parent = std::make_shared<const ManagedNode>(std::move(_parent));
        }

        ~ManagedNode() { // NOLINT(*-use-equals-default)

//                while (m_Parent && m_Parent.unique()) {
//                    m_Parent = std::move(m_Parent->m_Parent);
//                }

            Expunge_Recursive(m_Parent);
        }

        void Expunge_Recursive(std::shared_ptr<const ManagedNode>& _node) {
            if (_node && _node.unique()) {
                _node = std::move(_node->m_Parent);
                Expunge_Recursive(_node);
            }
        }
    };
}

#endif //CHDR_MANAGEDNODE_HPP