#ifndef CHDR_GRID_HPP
#define CHDR_GRID_HPP

#include "base/IMaze.hpp"
#include "../Node.hpp"

#include <Debug.hpp>

#include <vector>

namespace CHDR::Mazes {

    /**
     * @tparam Kd Dimensionality of the grid.
     */
    template <size_t Kd, typename T = uint32_t>
    class Grid : IMaze {
        
        static_assert(Kd > 0, "Kd must be greater than 0.");
        static_assert(std::is_integral_v<T>, "Type T must be integral.");

    private:

        std::vector<Node<T>> s_Nodes;

    public:

    };

} // CHDR::Mazes

#endif //CHDR_GRID_HPP