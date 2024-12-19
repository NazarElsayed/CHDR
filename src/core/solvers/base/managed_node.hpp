/*
* Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_MANAGEDNODE_HPP
#define CHDR_MANAGEDNODE_HPP

#include <cassert>
#include <cstddef>

#include "../../utils/intrinsics.hpp"
#include "bnode.hpp"

namespace chdr::solvers {

    template<typename index_t>
    struct managed_node : bnode<index_t> {

        managed_node* RESTRICT m_parent;
        unsigned char m_successors;

        /**
         * @brief Constructs an uninitialized ASNode.
         *
         * This constructor creates an ASNode with uninitialized members.
         */
        [[nodiscard]] constexpr managed_node() noexcept : // NOLINT(*-pro-type-member-init, *-use-equals-default)
            m_parent     (nullptr),
            m_successors (0U     ) {}

        [[nodiscard]] constexpr managed_node(const index_t& _index, managed_node* RESTRICT const _parent = nullptr) noexcept : bnode<index_t>(_index),
            m_parent     (_parent),
            m_successors (0U     )
        {
            incr();
        }

        [[nodiscard]] constexpr managed_node(const managed_node& _other) noexcept : bnode<index_t>(_other.m_index),
            m_parent     (_other.m_parent),
            m_successors (0U             )
        {
            incr();
        }

        managed_node& operator=(const managed_node& _other) noexcept {

            assert(this != &_other);

            expunge();

            bnode<index_t>::operator=(_other.m_index);

            m_parent     = _other.m_parent;
            m_successors = _other.m_successors;

            return *this;
        }

        [[nodiscard]] constexpr managed_node(managed_node&& _other) noexcept : bnode<index_t>(std::move(_other.m_index)),
            m_parent     (_other.m_parent    ),
            m_successors (_other.m_successors)
        {
            _other.m_parent     = nullptr;
            _other.m_successors = 0U;
        }

        constexpr managed_node& operator=(managed_node&& _other) noexcept {

            assert(this != &_other);

            expunge();

            bnode<index_t>::operator=(std::move(_other.m_index));

            m_parent     = _other.m_parent;
            m_successors = _other.m_successors;

            _other.m_parent     = nullptr;
            _other.m_successors = 0U;

            return *this;
        }

        virtual ~managed_node() noexcept {
            expunge();
        }

        void expunge() noexcept {

            while (m_parent != nullptr) {

                decr();

                if (m_parent->m_successors == 0U) {

                    auto* temp = m_parent;
                    m_parent = temp->m_parent;

                    temp->m_parent = nullptr;
                    delete temp;
                }
                else {
                    break;
                }
            }
        }

        constexpr void decr() noexcept {
            if (m_parent != nullptr) {
                assert(m_parent->m_successors != static_cast<decltype(m_successors)>(0U) && "Underflow detected!");
                m_parent->m_successors -= 1U;
            }
        }

        constexpr void incr() noexcept {
            if (m_parent != nullptr) {
                assert(m_parent->m_successors != static_cast<decltype(m_successors)>(-1U) && "Overflow detected!");
                m_parent->m_successors += 1U;
            }
        }

        template<typename node_t, typename coord_t>
        auto backtrack(const coord_t& _size, const size_t& _capacity = 0U) const {

            static_assert(std::is_base_of_v<managed_node, node_t>, "node_t must derive from managed_node");

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

        [[nodiscard]] friend constexpr bool operator < (const managed_node& _a, const managed_node& _b) noexcept {
            return _a.m_fScore == _b.m_fScore ?
                   _a.m_gScore >  _b.m_gScore :
                   _a.m_fScore >  _b.m_fScore;
        }
    };

} //chdr::solvers

#endif //CHDR_MANAGEDNODE_HPP