#ifndef CHDR_ISOLVER_HPP
#define CHDR_ISOLVER_HPP

namespace CHDR::Solvers {

    class ISolver {

    public:

        virtual void Solve(const Mazes::IMaze& _maze) = 0;

    };

} // CHDR::Solvers

#endif //CHDR_ISOLVER_HPP