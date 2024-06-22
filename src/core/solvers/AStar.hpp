#ifndef CHDR_ASTAR_HPP
#define CHDR_ASTAR_HPP

#include "base/ISolver.hpp"

#include <Debug.hpp>

namespace CHDR::Solvers {

    class AStar : ISolver {

    private:

    public:

        void Solve() override {
            throw std::runtime_error("Not implemented!");
        }

    };

} // CHDR::Solvers

#endif //CHDR_ASTAR_HPP