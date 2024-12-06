/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_INODE_HPP
#define CHDR_INODE_HPP

namespace chdr::mazes {

    class inode {

    public:

        virtual ~inode() = default;

        [[nodiscard]] virtual bool is_active() const = 0;
    };

} // chdr::mazes

#endif //CHDR_INODE_HPP