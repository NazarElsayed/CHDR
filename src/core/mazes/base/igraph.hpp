/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_IGRAPH_HPP
#define CHDR_IGRAPH_HPP

#include "IMaze.hpp"
#include "mazes/nodes/IDNode.hpp"

namespace chdr::mazes {

    template <typename index_t, typename scalar_t>
    class igraph : public IMaze<IDNode<index_t>, index_t> {

    public:

        using edge_t = std::pair<index_t, scalar_t>;

        template<typename... Args>
        [[nodiscard]]
        constexpr bool Contains(const Args&... _id) const {
            return Contains({ _id... });
        }
    };

} // CHDR::Mazes

#endif //CHDR_IGRAPH_HPP