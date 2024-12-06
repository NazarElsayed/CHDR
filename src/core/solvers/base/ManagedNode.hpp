/*
* Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_MANAGEDNODE_HPP
#define CHDR_MANAGEDNODE_HPP

#include "bNode.hpp"

#include <memory>

namespace chdr::solvers {

    template<typename index_t>
    struct ManagedNode : public bNode<index_t> {

        std::shared_ptr<const ManagedNode<index_t>> m_parent;

        /**
         * @brief Constructs an uninitialized ManagedNode.
         *
         * This constructor creates an ManagedNode with uninitialized members.
         */
        constexpr ManagedNode() {} // NOLINT(*-pro-type-member-init, *-use-equals-default)

        [[nodiscard]] constexpr ManagedNode(const index_t& _index) :
                bNode<index_t>(_index),
                m_parent() {}

        [[nodiscard]] constexpr ManagedNode(const index_t& _index, ManagedNode<index_t>&& _parent) :
                bNode<index_t>(_index)
        {
            m_parent = std::make_shared<const ManagedNode>(std::move(_parent));
        }

        ~ManagedNode() { // NOLINT(*-use-equals-default)

//                while (m_parent && m_parent.unique()) {
//                    m_parent = std::move(m_parent->m_parent);
//                }

            Expunge_Recursive(m_parent);
        }

        void Expunge_Recursive(std::shared_ptr<const ManagedNode>& _node) {
            if (_node && _node.unique()) {
                _node = std::move(_node->m_parent);
                Expunge_Recursive(_node);
            }
        }

        template<typename node_t, typename coord_t>
        void Backtrack(std::vector<coord_t>& _path, const coord_t& _size, const size_t& _capacity = 0U) {

            static_assert(std::is_base_of_v<ManagedNode<index_t>, node_t>, "node_t must derive from ManagedNode");

            const auto& curr = *static_cast<const node_t*>(this);

            // Recurse from end node to start node, inserting into a result buffer:
            _path.reserve(_capacity);
            _path.emplace_back(Utils::ToND(curr.m_index, _size));

            if (curr.m_parent != nullptr) {

                for (auto item = curr.m_parent; item->m_parent != nullptr;) {
                    _path.emplace_back(Utils::ToND(item->m_index, _size));

                    auto oldItem = item;
                    item = item->m_parent;
                    oldItem.reset();
                }
            }

            // Reverse the result:
            std::reverse(_path.begin(), _path.end());

        }
    };
}

#endif //CHDR_MANAGEDNODE_HPP