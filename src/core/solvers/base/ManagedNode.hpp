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

        std::shared_ptr<ManagedNode<index_t>> m_Parent;

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
            m_Parent = std::make_shared<ManagedNode>(std::move(_parent));
        }

        ~ManagedNode() { // NOLINT(*-use-equals-default)

//                while (m_Parent && m_Parent.unique()) {
//                    m_Parent = std::move(m_Parent->m_Parent);
//                }

            Expunge_Recursive(m_Parent);
        }

        void Expunge_Recursive(std::shared_ptr<ManagedNode>& _node) {
            if (_node && _node.unique()) {
                _node = std::move(_node->m_Parent);
                Expunge_Recursive(_node);
            }
        }

        template<typename TNode, typename coord_t>
        void Backtrack(std::vector<coord_t>& _path, const coord_t& _size, const size_t& _capacity = 0U) {

            static_assert(std::is_base_of_v<ManagedNode<index_t>, TNode>, "TNode must derive from ManagedNode");

            const auto& curr = *static_cast<const TNode*>(this);

            // Recurse from end node to start node, inserting into a result buffer:
            _path.reserve(curr.m_GScore);
            _path.emplace_back(Utils::ToND(curr.m_Index, _size));

            if (curr.m_Parent != nullptr) {

                for (auto& item = curr.m_Parent; item->m_Parent != nullptr;) {
                    _path.emplace_back(Utils::ToND(item->m_Index, _size));

                    auto oldItem = item;
                    item = item->m_Parent;
                    oldItem.reset();
                }
            }

            // Reverse the result:
            std::reverse(_path.begin(), _path.end());

        }
    };
}

#endif //CHDR_MANAGEDNODE_HPP