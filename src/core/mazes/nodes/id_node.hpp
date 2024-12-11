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

#include "base/inode.hpp"

namespace chdr::mazes {

    template <typename index_t = size_t>
    class id_node : inode {

        static_assert(std::is_integral_v<index_t>, "Type index_t must be an integral type.");

    private:

        index_t m_id;

    public:

        constexpr id_node(const index_t& _id) noexcept : m_id(_id) {}

        [[maybe_unused, nodiscard]] bool is_active() const noexcept override  {
            return true;
        }

        [[maybe_unused, nodiscard]] constexpr const index_t& id() const noexcept {
            return m_id;
        }

        [[maybe_unused]] constexpr void id(const index_t& _id) noexcept {
            m_id = _id;
        }
    };

} //chdr::mazes

#endif //CHDR_IDNODE_HPP