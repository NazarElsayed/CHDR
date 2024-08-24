#ifndef CHDR_DJIKSTRA_HPP
#define CHDR_DJIKSTRA_HPP

#include "base/ISolver.hpp"

#include <Debug.hpp>

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