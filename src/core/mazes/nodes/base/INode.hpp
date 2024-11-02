/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_INODE_HPP
#define CHDR_INODE_HPP

namespace CHDR::Mazes {

    class INode {

    public:

        [[nodiscard]] virtual bool IsActive() const = 0;
    };

} // CHDR

#endif //CHDR_INODE_HPP