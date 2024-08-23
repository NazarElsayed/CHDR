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