#ifndef CHDR_ASTAR_HPP
#define CHDR_ASTAR_HPP

#include "base/ISolver.hpp"

#include <Debug.hpp>

namespace CHDR::Solvers {

    template <typename T>
    class AStar : public ISolver<T> {

    private:

    public:

        void Solve(const Mazes::IMaze<T>& _maze) override {
            throw std::runtime_error("AStar::Solve(const Mazes::IMaze& _maze) Not implemented!");
        }

        template <size_t Kd>
        void Solve (const Mazes::Grid<Kd, T>& _maze) {

            Debug::Log("AStar::Solve(const Mazes::Grid<Kd, T>& _maze): A* grid solution unimplemented!", LogType::Warning);

            // A* grid code goes here...
        }

    };

} // CHDR::Solvers

#endif //CHDR_ASTAR_HPP