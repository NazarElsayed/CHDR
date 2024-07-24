#ifndef CHDR_ASTAR_HPP
#define CHDR_ASTAR_HPP

#include <cmath>

#include "base/ISolver.hpp"

#include <Debug.hpp>
#include <queue>

namespace CHDR::Solvers {

    template<typename T, size_t Kd>
    class AStar : public ISolver<T> {

    private:

        using coord_t = Coord<size_t, Kd>;

        struct ASData {
            float m_GScore;
            float m_FScore;

            coord_t m_Parent;

            constexpr ASData(const float& _gScore, const float& _hScore, const coord_t& _parent = {}) :
                m_GScore(_gScore),
                m_FScore(_gScore + _hScore),
                m_Parent(_parent) {}

        };

        struct ASNode {

            coord_t m_Coord;
            ASData  m_Data;

            ASNode(const coord_t& coord, const ASData& data) :
                m_Coord(coord),
                m_Data(data) {}
        };

        struct ASNodeCompare {
            bool operator()(const ASNode& _a, const ASNode& _b) {
                return _a.m_Data.m_FScore > _b.m_Data.m_FScore;
            }
        };

    public:

        void Solve(const Mazes::IMaze<T>& _maze) override {
            throw std::runtime_error("AStar::Solve(const Mazes::IMaze& _maze) Not implemented!");
        }

        auto Solve(const Mazes::Grid<Kd, T>& _maze, const coord_t& _start, const coord_t& _end) {
            std::vector<coord_t> result;

            std::priority_queue<ASNode, std::vector<ASNode>, ASNodeCompare> openSet;

            ASNode start(_start, ASData(0.0f, EuclideanDistance(_start, _end)));

            openSet.push(start);

            bool complete = false;
            while (!complete) {
                ASNode current = openSet.top();
                openSet.pop();

                if (current.m_Coord == _end) {
                    complete = true;
                    result.push_back(current.m_Coord);
                }
                else {
                    //Grab adjacent neighbours in each dimention. Does not include diagonals.
                    for (size_t i = 0U; i < Kd; i++) {
                        coord_t leftCoord  = current.m_Coord;
                        coord_t rightCoord = current.m_Coord;

                        --leftCoord[i];
                        ++rightCoord[i];

                        ASNode leftNeighbour(
                            leftCoord,
                            ASData(EuclideanDistance(current.m_Coord, leftCoord), EuclideanDistance(leftCoord, _end), current.m_Coord));

                        ASNode rightNeighbour(
                            rightCoord,
                            ASData(EuclideanDistance(current.m_Coord, rightCoord), EuclideanDistance(rightCoord, _end), current.m_Coord));

                        openSet.push(leftNeighbour);
                        openSet.push(rightNeighbour);
                    }
                }

                if (openSet.empty()) {
                    complete = true;
                }
            }

            //std::cout << result[0][0] << std::endl;

            return result;
        }

        static constexpr auto _distanceHeuristic(const coord_t& _A, const coord_t& _B) {
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
        static constexpr auto EuclideanDistance(const coord_t& _A, const coord_t& _B) {
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
        static constexpr auto SqrEuclideanDistance(const coord_t& _A, const coord_t& _B) {
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
        static constexpr auto ManhattanDistance(const coord_t& _A, const coord_t& _B) {
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
