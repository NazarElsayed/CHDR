#ifndef CHDR_TREE_HPP
#define CHDR_TREE_HPP

#include "base/IMaze.hpp"

#include <Debug.hpp>

namespace CHDR::Mazes {

    /**
     * @tparam Kd Dimensionality of the tree.
     */
    template <size_t Kd, typename T = uint32_t>
    class Tree : public IMaze<T> {

    private:

    public:

    };

} // CHDR::Mazes

#endif //CHDR_TREE_HPP