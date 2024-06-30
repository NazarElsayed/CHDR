#ifndef CHDR_ISOLVER_HPP
#define CHDR_ISOLVER_HPP

namespace CHDR::Solvers {

    template <typename T>
    class ISolver {

        static_assert(std::is_integral<T>::value, "T must be an integral type");

    public:

        virtual void Solve(const Mazes::IMaze<T>& _maze) = 0;

    };

} // CHDR::Solvers

#endif //CHDR_ISOLVER_HPP