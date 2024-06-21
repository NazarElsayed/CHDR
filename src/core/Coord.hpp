#ifndef CHDR_COORD_HPP
#define CHDR_COORD_HPP

#include <array>
#include <type_traits>

namespace CHDR {

    template<typename T, std::size_t Kd>
    using Coord = std::array<T, Kd>;

} // CHDR

#endif //CHDR_COORD_HPP