/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_TREE_HPP
#define CHDR_TREE_HPP

#include "base/IMaze.hpp"

#include <Debug.hpp>

namespace CHDR::Mazes {

    /**
     * @tparam Kd Dimensionality of the tree.
     */
    template <const size_t Kd, typename T = uint32_t>
    class Tree : public IMaze<IDNode<T>> {

    private:

    public:

    };

} // CHDR::Mazes

#endif //CHDR_TREE_HPP