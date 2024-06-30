#ifndef CHDR_DJIKSTRA_HPP
#define CHDR_DJIKSTRA_HPP

#include "base/ISolver.hpp"

#include <Debug.hpp>

namespace CHDR::Solvers {

    template <typename T>
    class Djikstra : public ISolver<T> {

    private:

    public:

        void Solve(const Mazes::IMaze<T>& _maze) override {
            throw std::runtime_error("Djikstra::Solve(const Mazes::IMaze& _maze) Not implemented!");
        }

    };

} // CHDR::Solvers

#endif //CHDR_DJIKSTRA_HPP