#ifndef CHDR_DJIKSTRA_HPP
#define CHDR_DJIKSTRA_HPP

#include "base/ISolver.hpp"

#include <Debug.hpp>

namespace CHDR::Solvers {

    class Djikstra : public ISolver {

    private:

    public:

        void Solve(const Mazes::IMaze& _maze) override {
            throw std::runtime_error("Not implemented!");
        }

    };

} // CHDR::Solvers

#endif //CHDR_DJIKSTRA_HPP