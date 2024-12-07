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

namespace chdr::mazes {

    template<typename node_t, typename index_t>
    class imaze {

        //static_assert(std::is_base_of<inode, node_t>::value, "node_t must derive from inode");

    public:

        virtual ~imaze() = default;

        [[nodiscard]] virtual
#if __cplusplus >= 202002L
        constexpr
#endif // __cplusplus >= 202002L
        bool contains(const index_t& _id) const = 0;

        [[nodiscard]] virtual
#if __cplusplus >= 202002L
        constexpr
#endif // __cplusplus >= 202002L
        size_t count() const = 0;

    };

} //chdr::mazes

#endif //CHDR_IMAZE_HPP