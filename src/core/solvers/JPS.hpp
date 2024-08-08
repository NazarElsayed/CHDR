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
            throw std::runtime_error("JPS::Solve(const Mazes::IMaze& _maze) Not implemented!");
        }
    };

} // CHDR::Solvers

#endif //CHDR_JPS_HPP