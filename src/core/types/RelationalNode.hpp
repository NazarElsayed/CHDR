/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_RELATIONALNODE_HPP
#define CHDR_RELATIONALNODE_HPP

#include "Node.hpp"

#include <vector>

namespace CHDR {

    template <typename W = bool>
    class RelationalNode : public Node<W> {

    private:

        std::vector<Node<W>> m_Neighbors;

    public:

        /**
         * @brief Sets the node's neighbors.
         * @param _end The new neighbors.
         */
        constexpr void Neighbors(const std::vector<Node<W>>& _end) {
            m_Neighbors = _end;
        }

        /**
         * @brief Returns the node's neighbors.
         * @return The node's neighbors.
         */
        [[nodiscard]] constexpr const std::vector<Node<W>>& Neighbors() {
            return m_Neighbors;
        }

        /**
         * @brief Returns the node's neighbors.
         * @return The node's neighbors.
         */
        [[nodiscard]] const std::vector<Node<W>>& Neighbors() const {
            return m_Neighbors;
        }
    };

} // CHDR

#endif //CHDR_RELATIONALNODE_HPP