#ifndef CHDR_UNMANAGEDNODE_HPP
#define CHDR_UNMANAGEDNODE_HPP

#include "../../utils/Intrinsics.hpp"
#include "BNode.hpp"

#include <memory>

namespace CHDR::Solvers {

    template<typename index_t>
    struct UnmanagedNode : public BNode<index_t> {

        const UnmanagedNode<index_t>* RESTRICT m_Parent;

        /**
        * @brief Constructs an uninitialized UnmanagedNode.
        *
        * This constructor creates an UnmanagedNode with uninitialized members.
        */
        constexpr UnmanagedNode() {} // NOLINT(*-pro-type-member-init, *-use-equals-default)

        [[nodiscard]] constexpr UnmanagedNode(const index_t& _index, const UnmanagedNode<index_t>* RESTRICT const _parent) :
            BNode<index_t>(_index), m_Parent(std::move(_parent)) {}


        ~UnmanagedNode() { // NOLINT(*-use-equals-default)

//                while (m_Parent && m_Parent.unique()) {
//                    m_Parent = std::move(m_Parent->m_Parent);
//                }

            Expunge_Recursive(m_Parent);
        }

        void Expunge_Recursive(std::shared_ptr<const UnmanagedNode>& _node) {
            if (_node && _node.unique()) {
                _node = std::move(_node->m_Parent);
                Expunge_Recursive(_node);
            }
        }
    };
}

#endif //CHDR_UNMANAGEDNODE_HPP