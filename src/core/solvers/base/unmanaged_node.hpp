#ifndef CHDR_UNMANAGEDNODE_HPP
#define CHDR_UNMANAGEDNODE_HPP

#include <cstddef>
#include <memory>

#include "../../utils/intrinsics.hpp"
#include "bnode.hpp"

namespace chdr::solvers {

    template<typename index_t>
    struct unmanaged_node : bnode<index_t> {

        const unmanaged_node* RESTRICT m_parent;

        /**
        * @brief Constructs an uninitialized UnmanagedNode.
        *
        * This constructor creates an UnmanagedNode with uninitialized members.
        */
        // ReSharper disable once CppPossiblyUninitializedMember
        constexpr unmanaged_node() {} // NOLINT(*-pro-type-member-init, *-use-equals-default)

        [[nodiscard]] constexpr unmanaged_node(const index_t& _index, const unmanaged_node* RESTRICT const _parent) :
                bnode<index_t>(_index), m_parent(std::move(_parent)) {}

        template<typename node_t, typename coord_t>
        auto backtrack(std::vector<coord_t>& _path, const coord_t& _size, const size_t& _capacity = 0U) {

            static_assert(std::is_base_of_v<unmanaged_node, node_t>, "node_t must derive from UnmanagedNode");

            // Reserve space in result:
            _path.reserve(_capacity);

            // Recurse from end node to start node, inserting into a result buffer:
            #pragma unroll
            for (const auto* temp = this; temp->m_parent != nullptr; temp = static_cast<const node_t*>(temp->m_parent)) {
                _path.emplace_back(utils::to_nd(temp->m_index, _size));
            }

            // Reverse the result:
            std::reverse(_path.begin(), _path.end());
        }
    };
}

#endif //CHDR_UNMANAGEDNODE_HPP