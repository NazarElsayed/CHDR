#ifndef CHDR_GRAPH_HPP
#define CHDR_GRAPH_HPP

#include "base/IMaze.hpp"
#include "types/RelationalNode.hpp"
#include "types/Way.hpp"

#include <Debug.hpp>

namespace CHDR::Mazes {

    template <typename T>
    class Graph : public IMaze<T> {

    private:

        std::unordered_map<RelationalNode<T>, Way<T>> m_Ways;

    public:



    };

} // CHDR::Mazes

#endif //CHDR_GRAPH_HPP