/*
* Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_BNODE_HPP
#define CHDR_BNODE_HPP

#include <cassert>

namespace chdr::solvers {

    template<typename index_t>
    struct bnode {

        index_t m_index;

        /**
         * @brief Constructs an uninitialized BNode.
         *
         * This constructor creates an BNode with uninitialized members.
         */
        // ReSharper disable once CppPossiblyUninitializedMember
        constexpr bnode() noexcept {} // NOLINT(*-pro-type-member-init, *-use-equals-default)

        constexpr bnode(const index_t& _index) noexcept : m_index(_index) {}

        constexpr bnode(const bnode& _other) noexcept : m_index(_other.m_index) {}

        constexpr bnode(bnode&& _other) noexcept : m_index(std::move(_other.m_index)) {}

        virtual ~bnode() noexcept = default;
        
        bnode& operator=(const bnode& _other) noexcept {

            assert(this != &_other && "Self-assignment detected.");

            m_index = _other.m_index;
            return *this;
        }

        bnode& operator=(bnode&& _other) noexcept {

            assert(this != &_other && "Self-assignment detected.");

            m_index = std::move(_other.m_index);
            return *this;
        }

        [[nodiscard]] friend constexpr bool operator == (const bnode& _a, const bnode& _b) noexcept { return _a.m_index == _b.m_index; }

    };

} //chdr::solvers

#endif //CHDR_BNODE_HPP