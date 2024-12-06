#ifndef CHDR_UNMANAGEDNODE_HPP
#define CHDR_UNMANAGEDNODE_HPP

#include "../../utils/Intrinsics.hpp"
#include "bNode.hpp"

#include <memory>

namespace chdr::solvers {

    template<typename index_t>
    struct UnmanagedNode : public bNode<index_t> {

        const UnmanagedNode<index_t>* RESTRICT m_parent;

        /**
        * @brief Constructs an uninitialized UnmanagedNode.
        *
        * This constructor creates an UnmanagedNode with uninitialized members.
        */
        constexpr UnmanagedNode() {} // NOLINT(*-pro-type-member-init, *-use-equals-default)

        [[nodiscard]] constexpr UnmanagedNode(const index_t& _index, const UnmanagedNode<index_t>* RESTRICT const _parent) :
                bNode<index_t>(_index), m_parent(std::move(_parent)) {}

        template<typename node_t, typename coord_t>
        auto Backtrack(std::vector<coord_t>& _path, const coord_t& _size, const size_t& _capacity = 0U) {

            static_assert(std::is_base_of_v<UnmanagedNode<index_t>, node_t>, "node_t must derive from UnmanagedNode");

            // Reserve space in result:
            _path.reserve(_capacity);

            // Recurse from end node to start node, inserting into a result buffer:
            for (const auto* temp = this; temp->m_parent != nullptr; temp = static_cast<const node_t*>(temp->m_parent)) {
                _path.emplace_back(Utils::ToND(temp->m_index, _size));
            }

            // Reverse the result:
            std::reverse(_path.begin(), _path.end());
        }
    };
}

#endif //CHDR_UNMANAGEDNODE_HPP