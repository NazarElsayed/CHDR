/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_IDNODE_HPP
#define CHDR_IDNODE_HPP

#include <cstddef>
#include <type_traits>

#include "base/INode.hpp"

namespace chdr::mazes {

    template <typename index_t = size_t>
    class IDNode : INode {

        static_assert(std::is_integral_v<index_t>, "Type index_t must be an integral type.");

    private:

        index_t m_ID;

    public:

        constexpr IDNode(const index_t& _id) : m_ID(_id) {}

        [[maybe_unused]] [[nodiscard]] bool IsActive() const override  {
            return true;
        }

        [[maybe_unused]] [[nodiscard]] constexpr const index_t& ID() const {
            return m_ID;
        }

        [[maybe_unused]] constexpr void ID(const index_t& _id) {
            m_ID = _id;
        }
    };

} // CHDR

#endif //CHDR_IDNODE_HPP