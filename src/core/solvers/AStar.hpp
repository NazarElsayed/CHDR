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
        void Solve (const Mazes::Grid<Kd, T>& _maze, const Coord<size_t, Kd>& _start, const Coord<size_t, Kd>& _end) {

			std::vector<int> result;

			_start.m_Parent = nullptr;
			_start.m_GScore = 0;
			_start.m_FScore = EulerDistance(_start, _target);

			// Custom comparitor for open set. Orders the set like a min heap.
			auto min = [](ASNode* _A, ASNode* _B) { return _A->m_FScore > _B->m_FScore; };

			// Create a priority queue to contain nodes as we process them.
			std::priority_queue <ASNode*, std::vector<ASNode*>, decltype(min)> workingSet(min);
			workingSet.push(&_start);

			/* Solve the path */
			ASNode* curr = nullptr;

			bool solutionFound = false;

			while (workingSet.size() != 0) {

				curr = workingSet.top();

				if (curr == &_target) {
					solutionFound = true;
					break;
				}

				workingSet.pop();

				for (auto& neighbor : curr->m_Neighbors) {

					auto gScore = curr->m_GScore + _distanceHeuristic(*curr, *neighbor);

					if (gScore < neighbor->m_GScore) {
						neighbor->m_Parent = curr;
						neighbor->m_GScore = gScore;
						neighbor->m_FScore = gScore + _distanceHeuristic(*neighbor, _target);

						workingSet.push(neighbor);
					}
				}
			}

			if (solutionFound) {

				while (curr != nullptr) {
					result.insert(result.end(), curr->m_ID);

					curr = curr->m_Parent;
				}
			}

			return result;
        }

        struct ASNode {

			public:
				ASNode* m_Parent;

				std::vector<ASNode*> m_Neighbors;

				int m_ID;
				float m_Position[2];

				float m_GScore;
				float m_FScore;

				ASNode() {
					m_GScore = FLT_MAX;
				}
		};

    };
} // CHDR::Solvers

#endif //CHDR_ASTAR_HPP