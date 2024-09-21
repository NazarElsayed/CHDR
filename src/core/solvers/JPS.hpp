/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_JPS_HPP
#define CHDR_JPS_HPP

#include "base/ISolver.hpp"

#include <Debug.hpp>

namespace CHDR::Solvers {

    template<typename T>
    class JPS final : public ISolver<T> {

    private:

    public:

        void Solve(const Mazes::IMaze<T>& _maze) override {

            (void)_maze; // Suppress unused variable warning.
            
            throw std::runtime_error("JPS::Solve(const Mazes::IMaze& _maze) Not implemented!");
        }
    };

} // CHDR::Solvers

#endif //CHDR_JPS_HPP