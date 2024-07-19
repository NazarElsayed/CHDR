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
    	struct ASData {

    		Coord<size_t, Kd> m_Parent;

    		float m_GScore;
    		float m_FScore;

    		constexpr ASData(const Coord<size_t, Kd>& _parent, const float& _gScore, const float& _fScore) :
    			m_Parent(_parent),
    			m_GScore(_gScore),
    			m_FScore(_fScore) {}
    	};

    public:

        void Solve(const Mazes::IMaze<T>& _maze) override {
            throw std::runtime_error("AStar::Solve(const Mazes::IMaze& _maze) Not implemented!");
        }

        template <size_t Kd>
        auto Solve(const Mazes::Grid<Kd, T>& _maze, const Coord<size_t, Kd>& _start, const Coord<size_t, Kd>& _end) {

			using data_t = ASData<Kd>;
        	using coord_t = Coord<size_t, Kd>;

        	std::vector<coord_t> result;

			std::unordered_map<coord_t, data_t> openSet;

			//TODO: The squiglies are bad.
			data_t startData({}, 0.0f, EuclideanDistance(_start, _end));
        	std::pair<coord_t, data_t> start = std::make_pair(_start, startData);

        	openSet.insert(start);

			// Custom comparitor for open set. Orders the set like a min heap.
			auto min = [](data_t* _A, data_t* _B) { return _A->m_FScore > _B->m_FScore; };

			// Create a priority queue to contain nodes as we process them.
			std::priority_queue<data_t, decltype(min)> workingSet(min);
			workingSet.push(start);

			/* Solve the path */
			data_t* curr = nullptr;

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
		static constexpr auto _distanceHeuristic(const Coord<size_t, Kd>& _A, const Coord<size_t, Kd>& _B) {
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
    	static constexpr auto EuclideanDistance(const Coord<size_t, Kd>& _A, const Coord<size_t, Kd>& _B) {
        	return sqrtf(SqrEuclideanDistance(_A, _B));
        }

    	/*
		 * @brief Computes the squared Euclidean distance between two nodes.
		 *
		 * @tparam Kd The number of dimensions of the nodes.
		 * @param _A The first node.
		 * @param _B The second node.
		 * @return The squared Euclidean distance between _A and _B.
		 */
    	template <size_t Kd>
    	static constexpr auto SqrEuclideanDistance(const Coord<size_t, Kd>& _A, const Coord<size_t, Kd>& _B) {

			float result = 0.0F;

        	for (size_t i = 0U; i < Kd; ++i) {

				auto val = _B[i] - _A[i];
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
    	static constexpr auto ManhattanDistance(const Coord<size_t, Kd>& _A, const Coord<size_t, Kd>& _B) {

        	float result = 0.0F;

        	for (size_t i = 0U; i < Kd; ++i) {
        		result += abs(_B[i] - _A[i]);
        	}

        	return result;
        }

#pragma endregion Heuristics

    };
} // CHDR::Solvers

#endif //CHDR_ASTAR_HPP