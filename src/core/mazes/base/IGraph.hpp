/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_IGRAPH_HPP
#define CHDR_IGRAPH_HPP

#include "../../nodes/IDNode.hpp"

#include "IMaze.hpp"

#include <unordered_set>

namespace CHDR::Mazes {

    template <typename Ti, typename Ts = float>
    class IGraph : IMaze<IDNode<Ti>, Ti> {

    public:

        using edge_t = std::pair<Ti, Ts>;

        template<typename... Args>
        [[nodiscard]]
        constexpr bool Contains(const Args&... _id) const {
            return Contains({ _id... });
        }

        [[nodiscard]] virtual constexpr bool Contains(const Ti& _id) const = 0;

    };

} // CHDR::Mazes

#endif //CHDR_IGRAPH_HPP