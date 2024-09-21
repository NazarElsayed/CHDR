/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_GRAPH_HPP
#define CHDR_GRAPH_HPP

#include "base/IMaze.hpp"
#include "types/RelationalNode.hpp"
#include "types/Way.hpp"

#include <Debug.hpp>

namespace CHDR::Mazes {

    template <typename T>
    class Graph : public IMaze<T> {

    private:

        std::unordered_map<RelationalNode<T>, Way<T>> m_Ways;

    public:

    };

} // CHDR::Mazes

#endif //CHDR_GRAPH_HPP