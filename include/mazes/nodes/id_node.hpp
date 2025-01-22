/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_IDNODE_HPP
#define CHDR_IDNODE_HPP

#include <type_traits>

namespace chdr::mazes {

    template <typename index_t>
    class id_node final {

        static_assert(std::is_integral_v<index_t>, "Type index_t must be an integral type.");

    private:

        index_t m_id;

    public:

        [[nodiscard]] constexpr id_node(index_t _id) noexcept : m_id(_id) {}
        ~id_node() noexcept = default;

        [[nodiscard]] id_node           (const id_node&) = delete;
                      id_node& operator=(const id_node&) = delete;

        [[nodiscard]] constexpr id_node(id_node&&) noexcept = default;

#if __cplusplus > 202302L
        constexpr
#endif
        id_node& operator=(id_node&&) noexcept = default;
        
        [[maybe_unused, nodiscard]] constexpr static bool is_active() noexcept { return true; }

        [[maybe_unused, nodiscard]] constexpr index_t id() const noexcept { return m_id; }

        [[maybe_unused]] constexpr void id(index_t _id) noexcept { m_id = _id; }
    };

} //chdr::mazes

#endif //CHDR_IDNODE_HPP