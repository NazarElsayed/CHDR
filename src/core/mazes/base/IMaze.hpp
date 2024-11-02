/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_IMAZE_HPP
#define CHDR_IMAZE_HPP

#include <cstddef>

namespace CHDR::Mazes {

    template<typename TNode, typename index_t>
    class IMaze {

        //static_assert(std::is_base_of<INode, TNode>::value, "TNode must derive from INode");

    public:

        [[nodiscard]] virtual
#if __cplusplus >= 202002L
        constexpr
#endif // __cplusplus >= 202002L
        bool Contains(const index_t& _id) const = 0;

        [[nodiscard]] virtual
#if __cplusplus >= 202002L
        constexpr
#endif // __cplusplus >= 202002L
        size_t Count() const = 0;

    };

} // CHDR::Mazes

#endif //CHDR_IMAZE_HPP