/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_DJIKSTRA_HPP
#define CHDR_DJIKSTRA_HPP

#include "base/ISolver.hpp"

namespace CHDR::Solvers {

    template <typename T>
    class Dijkstra final : public ISolver<T> {

    private:

    public:

        void Solve(const Mazes::IMaze<T>& _maze) override {

            (void)_maze; // Suppress unused variable warning.

            throw std::runtime_error("Djikstra::Solve(const Mazes::IMaze& _maze) Not implemented!");
        }

    };

} // CHDR::Solvers

#endif //CHDR_DJIKSTRA_HPP