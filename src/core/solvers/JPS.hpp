#ifndef CHDR_JPS_HPP
#define CHDR_JPS_HPP

#include "base/ISolver.hpp"

#include <Debug.hpp>

namespace CHDR::Solvers {

    class JPS : public ISolver {

    private:

    public:

        void Solve(const Mazes::IMaze& _maze) override {
            throw std::runtime_error("Not implemented!");
        }
    };

} // CHDR::Solvers

#endif //CHDR_JPS_HPP