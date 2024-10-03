/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_IMAZE_HPP
#define CHDR_IMAZE_HPP

#include "../../types/Node.hpp"

namespace CHDR::Mazes {

    template <typename T>
    class IMaze {

    public:

        [[nodiscard]] virtual constexpr const Node<T>& At(const size_t& _index) const = 0;

        [[nodiscard]] virtual constexpr bool Contains(const size_t& _id) const = 0;

    };

} // CHDR::Mazes

#endif //CHDR_IMAZE_HPP