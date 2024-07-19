#ifndef CHDR_ASTAR_HPP
#define CHDR_ASTAR_HPP

#include <cmath>

#include "base/ISolver.hpp"

#include <Debug.hpp>
#include <queue>

namespace CHDR::Solvers {

    template <typename T>
    class AStar : public ISolver<T> {

    private:

    	template <size_t Kd>
    	struct ASNode {

    		ASNode* m_Parent;

    		std::vector<ASNode*> m_Neighbors;

			Coord<size_t, Kd> m_Position;

    		float m_GScore;
    		float m_FScore;

    		ASNode() {
    			m_GScore = std::numeric_limits<float>::max();
    		}
    	};

    public:

        void Solve(const Mazes::IMaze<T>& _maze) override {
            throw std::runtime_error("AStar::Solve(const Mazes::IMaze& _maze) Not implemented!");
        }

        template <size_t Kd>
        auto Solve(const Mazes::Grid<Kd, T>& _maze, const Coord<size_t, Kd>& _start, const Coord<size_t, Kd>& _end) {

			using node_t = ASNode<Kd>;
        	using coord_t = Coord<size_t, Kd>;

			std::vector<coord_t> result;

			_start.m_Parent = nullptr;
			_start.m_GScore = 0;
			_start.m_FScore = EulerDistance(_start, _end);

			// Custom comparitor for open set. Orders the set like a min heap.
			auto min = [](node_t* _A, node_t* _B) { return _A->m_FScore > _B->m_FScore; };

			// Create a priority queue to contain nodes as we process them.
			std::priority_queue<node_t*, std::vector<node_t*>, decltype(min)> workingSet(min);
			workingSet.push(&_start);

			/* Solve the path */
			node_t* curr = nullptr;

			bool solutionFound = false;

			while (workingSet.size() != 0) {

				curr = workingSet.top();

				if (curr == &_end) {
					solutionFound = true;
					break;
				}

				workingSet.pop();

				for (auto& neighbor : curr->m_Neighbors) {

					auto gScore = curr->m_GScore + _distanceHeuristic(*curr, *neighbor);

					if (gScore < neighbor->m_GScore) {
						neighbor->m_Parent = curr;
						neighbor->m_GScore = gScore;
						neighbor->m_FScore = gScore + _distanceHeuristic(*neighbor, _end);

						workingSet.push(neighbor);
					}
				}
			}

			if (solutionFound) {

				while (curr != nullptr) {
					result.insert(result.end(), curr->m_Position);

					curr = curr->m_Parent;
				}
			}

        	return result;
        }

    	template <size_t Kd>
		static auto _distanceHeuristic(const ASNode<Kd>& _A, const ASNode<Kd>& _B) {
	        return ManhattanDistance(_A, _B);
        }

#pragma region Heuristics
    	
    	/**
		 * @brief Computes the Euclidean distance between two nodes.
         *
    	 * @tparam Kd The number of dimensions of the nodes.
    	 * @param _A The first node.
		 * @param _B The second node.
		 * @return The Euclidean distance between _A and _B.
		 */
    	template <size_t Kd>
    	static auto EuclideanDistance(const ASNode<Kd>& _A, const ASNode<Kd>& _B) {
        	return sqrt(SqrEulerDistance(_A, _B));
        }

    	/**
		 * @brief Computes the squared Euclidean distance between two nodes.
		 *
		 * @tparam Kd The number of dimensions of the nodes.
		 * @param _A The first node.
		 * @param _B The second node.
		 * @return The squared Euclidean distance between _A and _B.
		 */
    	template <size_t Kd>
    	static auto SqrEuclideanDistance(const ASNode<Kd>& _A, const ASNode<Kd>& _B) {

			float result = 0.0F;

        	for (auto i = 0U; i < Kd; ++i) {

				auto val = _B.m_Position[i] - _A.m_Position[i];
        		result += val * val;
        	}

        	return result;
        }

		/**
		  * @brief Calculate the Manhattan distance between two nodes.
		  *
		  * @tparam Kd The number of dimensions of the nodes.
		  * @param _A The first node.
		  * @param _B The second node.
		  * @return The Manhattan distance between _A and _B.
		  */
    	template <size_t Kd>
    	static auto ManhattanDistance(const ASNode<Kd>& _A, const ASNode<Kd>& _B) {

        	float result = 0.0F;

        	for (auto i = 0U; i < Kd; ++i) {
        		result += abs(_B.m_Position[i] - _A.m_Position[i]);
        	}

        	return result;
        }

#pragma endregion Heuristics

    };
} // CHDR::Solvers

#endif //CHDR_ASTAR_HPP