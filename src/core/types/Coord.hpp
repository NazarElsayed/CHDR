#ifndef CHDR_COORD_HPP
#define CHDR_COORD_HPP

#include <cstddef>
#include <array>
#include <type_traits>

namespace CHDR {

    template<typename T, size_t Kd>
    using Coord = std::array<T, Kd>;

} // CHDR

namespace std {

    template<typename T, size_t Kd>
    struct hash<CHDR::Coord<T, Kd>> {

        size_t operator()(const CHDR::Coord<T, Kd>& _value) const {

            size_t result = 0U;

            std::hash<T> hashing_func;

            for (size_t i = 0U; i < Kd; ++i) {
                result ^= hashing_func(_value[i]) << (i % (sizeof(size_t)*8));
            }

            return result;
        }
    };

} // std

#endif //CHDR_COORD_HPP