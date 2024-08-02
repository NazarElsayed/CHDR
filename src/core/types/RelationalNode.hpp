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

        RelationalNode(const W& _value, const std::vector<Node<W>> _neighbors) :
            Node<W>(_value),
            m_Neighbors(_neighbors) {};

    };

} // CHDR

#endif //CHDR_RELATIONALNODE_HPP