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
            m_Parent = std::make_shared(std::move(_parent));
        }

    };
}

#endif //CHDR_MANAGEDNODE_HPP