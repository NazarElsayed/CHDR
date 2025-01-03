/*
* Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_UNMANAGEDNODE_HPP
#define CHDR_UNMANAGEDNODE_HPP

#include <cstddef>
#include <vector>

#include "../../utils/intrinsics.hpp"
#include "bnode.hpp"

namespace chdr::solvers {

    template<typename index_t>
    struct unmanaged_node : bnode<index_t> {

        const unmanaged_node* RESTRICT m_parent;

        /**
         * @brief Constructs an uninitialized node.
         *
         * This constructor creates a node with uninitialized members.
         */
        // ReSharper disable once CppPossiblyUninitializedMember
        constexpr unmanaged_node() noexcept {} // NOLINT(*-pro-type-member-init, *-use-equals-default)

        [[nodiscard]] constexpr unmanaged_node(const index_t& _index, const unmanaged_node* RESTRICT const _parent = nullptr) noexcept : bnode<index_t>(_index),
            m_parent(std::move(_parent)) {}

        template<typename node_t, typename coord_t>
        auto backtrack(const coord_t& _size, const size_t& _capacity = 1U) const {

            static_assert(std::is_base_of_v<unmanaged_node, node_t>, "node_t must derive from unmanaged_node");

            // Reserve space in result:
            std::vector<coord_t> result;
            result.reserve(_capacity);

            // Recurse from end node to start node, inserting into a result buffer:
            for (const auto* t = this; t->m_parent != nullptr; t = static_cast<const node_t*>(t->m_parent)) {
                result.emplace_back(utils::to_nd(t->m_index, _size));
            }

            // Reverse the result:
            std::reverse(result.begin(), result.end());

            return result;
        }
    };
}

#endif //CHDR_UNMANAGEDNODE_HPP