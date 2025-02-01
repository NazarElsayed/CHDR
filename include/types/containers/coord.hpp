/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_COORD_HPP
#define CHDR_COORD_HPP

/**
 * @file coord.hpp
 *
 * @details K-dimensional coordinate type.
 */

#include <array>
#include <cstddef>

namespace chdr {

    // ReSharper disable CppInconsistentNaming

    /**
      * @brief A K-dimensional coordinate type.
      * @details A type for representing points in a K-dimensional space using `std::array`.
      * @remarks This type supports use in a `constexpr` context.
      * @tparam T Coordinate element type.
      * @tparam Kd Dimensionality of the coordinate.
      */
    template <typename T, size_t Kd>
    using coord [[maybe_unused]] = std::array<T, Kd>;

    // ReSharper restore CppInconsistentNaming

} //chdr

#endif //CHDR_COORD_HPP