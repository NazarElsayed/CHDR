/*
* Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_MANAGEDNODE_HPP
#define CHDR_MANAGEDNODE_HPP

#include <cstddef>
#include <memory>

#include "bnode.hpp"

namespace chdr::solvers {

    template<typename index_t>
    struct managed_node : bnode<index_t> {

        std::shared_ptr<const managed_node> m_parent;

        /**
         * @brief Constructs an uninitialized ManagedNode.
         *
         * This constructor creates an managed_node with uninitialized members.
         */
        // ReSharper disable once CppPossiblyUninitializedMember
        constexpr managed_node() {} // NOLINT(*-pro-type-member-init, *-use-equals-default)

        [[nodiscard]] constexpr managed_node(const index_t& _index) :
                bnode<index_t>(_index),
                m_parent() {}

        [[nodiscard]] constexpr managed_node(const index_t& _index, managed_node&& _parent) :
                bnode<index_t>(_index), m_parent(std::make_shared<const managed_node>(std::move(_parent))) {}

        ~managed_node() { // NOLINT(*-use-equals-default)

#ifdef __OPTIMIZE__
            Expunge_Recursive(m_parent);    // Only attempt TRO if compiled with optimisation flags.
#else //!__OPTIMIZE__
            while (m_parent && m_parent.unique()) {
                m_parent = std::move(m_parent->m_parent);
            }
#endif//!__OPTIMIZE__

        }

#pragma inline_recursion(on)

        void expunge_recursive(std::shared_ptr<const managed_node>& _node) {
            if (_node && _node.unique()) {
                _node = std::move(_node->m_parent);
                Expunge_Recursive(_node);
            }
        }

#pragma inline_recursion(off)

        template<typename node_t, typename coord_t>
        void backtrack(std::vector<coord_t>& _path, const coord_t& _size, const size_t& _capacity = 0U) {

            static_assert(std::is_base_of_v<managed_node, node_t>, "node_t must derive from managed_node");

            const auto& curr = *static_cast<const node_t*>(this);

            // Recurse from end node to start node, inserting into a result buffer:
            _path.reserve(_capacity);
            _path.emplace_back(utils::to_nd(curr.m_index, _size));

            if (curr.m_parent != nullptr) {

                #pragma unroll
                for (auto item = curr.m_parent; item->m_parent != nullptr;) {
                    _path.emplace_back(utils::to_nd(item->m_index, _size));

                    auto oldItem = item;
                    item = item->m_parent;
                    oldItem.reset();
                }
            }

            // Reverse the result:
            std::reverse(_path.begin(), _path.end());

        }
    };

} //chdr::solvers

#endif //CHDR_MANAGEDNODE_HPP