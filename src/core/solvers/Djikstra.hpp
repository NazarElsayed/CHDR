#ifndef CHDR_DJIKSTRA_HPP
#define CHDR_DJIKSTRA_HPP

#include "base/ISolver.hpp"

#include <Debug.hpp>

namespace CHDR::Solvers {

    class Djikstra : ISolver {

    private:

    public:

        void Solve() override {
            throw std::runtime_error("Not implemented!");
        }

    };

} // CHDR::Solvers

#endif //CHDR_DJIKSTRA_HPP