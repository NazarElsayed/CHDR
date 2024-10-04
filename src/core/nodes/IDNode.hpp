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

#include "nodes/base/INode.hpp"

namespace CHDR {

    template <typename Ti = size_t>
    class IDNode : INode {

        static_assert(std::is_integral_v<Ti>, "Type Ti must be an integral type.");

    private:

        Ti m_ID;

    public:

        constexpr IDNode(const Ti& _id) : m_ID(_id) {}

        [[nodiscard]] bool IsActive() const override  {
            return true;
        }

        [[nodiscard]] constexpr const Ti& ID() const {
            return m_ID;
        }

        constexpr void ID(const Ti& _id) {
            m_ID = _id;
        }
    };

} // CHDR

#endif //CHDR_IDNODE_HPP