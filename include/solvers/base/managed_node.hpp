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

#include "../../types/pmr/polytonic_pool.hpp"
#include "bnode.hpp"

// ReSharper disable once CppUnusedIncludeDirective
#include "../../utils/intrinsics.hpp" // NOLINT(*-include-cleaner)

namespace chdr::solvers {

    template <typename index_t, typename derived = void>
    struct managed_node : bnode<index_t> {

        using node_t = std::conditional_t<std::is_void_v<derived>, managed_node<index_t>, derived>;

        managed_node* RESTRICT m_parent;
        unsigned char m_successors;

        /**
         * @brief Constructs an uninitialized node.
         *
         * This constructor creates a node with uninitialized members.
         */
        [[nodiscard]] constexpr managed_node() noexcept : // NOLINT(*-pro-type-member-init, *-use-equals-default)
            m_parent(nullptr),
            m_successors(0U) {}
        ~managed_node() noexcept = default;

        [[nodiscard]] constexpr managed_node(const index_t& _index, managed_node* RESTRICT const _parent = nullptr) noexcept : bnode<index_t>(_index),
            m_parent(_parent),
            m_successors(0U)
        {
            if (_parent != nullptr) {
                incr();
            }
        }

        constexpr managed_node           (const managed_node&) = delete;
        constexpr managed_node& operator=(const managed_node&) = delete;

        [[nodiscard]] constexpr managed_node(managed_node&&) noexcept = default;

#if __cplusplus > 202302L
        constexpr
#endif
        managed_node& operator=(managed_node&&) noexcept = default;

        template <typename pmr_t>
        void expunge(pmr_t* _pmr) noexcept {

            while (m_parent != nullptr) {
                decr();

                if (m_parent->m_successors != 0U) {
                    break;
                }
                
                auto* const RESTRICT d = m_parent;
                m_parent = m_parent->m_parent;
                _pmr->deallocate(d, sizeof(managed_node), alignof(managed_node));
            }
        }

        constexpr void decr() const noexcept {
            assert(m_parent != nullptr && "Dereferencing of nullptr.");
            assert(m_parent->m_successors != static_cast<decltype(m_successors)>(0U) && "Underflow detected!");

            --m_parent->m_successors;
        }

        constexpr void incr() const noexcept {
            assert(m_parent != nullptr && "Dereferencing of nullptr.");
            assert(m_parent->m_successors != static_cast<decltype(m_successors)>(-1U) && "Overflow detected!");

            ++m_parent->m_successors;
        }

        [[nodiscard]] friend constexpr bool operator < (const managed_node& _a, const managed_node& _b) noexcept {
            return _a.m_fScore == _b.m_fScore ?
                   _a.m_gScore >  _b.m_gScore :
                   _a.m_fScore >  _b.m_fScore;
        }
    };

} //chdr::solvers

#endif //CHDR_MANAGEDNODE_HPP