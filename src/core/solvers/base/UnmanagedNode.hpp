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

        template<typename TNode, typename coord_t>
        auto Backtrack(std::vector<coord_t>& _path, const coord_t& _size, const size_t& _capacity = 0U) {

            static_assert(std::is_base_of_v<UnmanagedNode<index_t>, TNode>, "TNode must derive from UnmanagedNode");

            // Reserve space in result:
            _path.reserve(_capacity);

            // Recurse from end node to start node, inserting into a result buffer:
            for (const auto* temp = &this; temp->m_Parent != nullptr; temp = static_cast<const TNode*>(temp->m_Parent)) {
                _path.emplace_back(Utils::ToND(temp->m_Index, _size));
            }

            // Reverse the result:
            std::reverse(_path.begin(), _path.end());
        }
    };
}

#endif //CHDR_UNMANAGEDNODE_HPP