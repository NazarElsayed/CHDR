/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_COORD_HPP
#define CHDR_COORD_HPP

#include <array>

namespace chdr {

    template<typename T, const size_t Kd>
    using coord [[maybe_unused]] = std::array<T, Kd>;

} // chdr

#endif //CHDR_COORD_HPP