/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_IMAZE_HPP
#define CHDR_IMAZE_HPP

#include "../../nodes/base/INode.hpp"

namespace CHDR::Mazes {

    template<typename TNode, typename Ti>
    class IMaze {

        //static_assert(std::is_base_of<INode, TNode>::value, "TNode must derive from INode");

    public:

        [[nodiscard]] virtual constexpr bool Contains(const Ti& _id) const = 0;

        [[nodiscard]] virtual constexpr size_t Count() const = 0;

    };

} // CHDR::Mazes

#endif //CHDR_IMAZE_HPP