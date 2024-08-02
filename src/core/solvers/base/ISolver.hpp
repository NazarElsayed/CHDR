#ifndef CHDR_ISOLVER_HPP
#define CHDR_ISOLVER_HPP

namespace CHDR::Solvers {

    template <typename Tm>
    class ISolver {

        static_assert(std::is_integral<Tm>::value, "Tm must be an integral type");

    public:

        virtual void Solve(const Mazes::IMaze<Tm>& _maze) = 0;

    };

} // CHDR::Solvers

#endif //CHDR_ISOLVER_HPP