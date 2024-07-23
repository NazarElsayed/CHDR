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

            coord_t coord;
            ASData  data;

            ASNode(const coord_t& coord, const ASData& data) :
                coord(coord),
                data(data) {}
        };

        struct ASNodeCompare {
            bool operator()(const ASNode& _a, const ASNode& _b) {
                return _a.data.m_FScore > _b.data.m_FScore;
            }
        };

    public:

        void Solve(const Mazes::IMaze<T>& _maze) override {
            throw std::runtime_error("AStar::Solve(const Mazes::IMaze& _maze) Not implemented!");
        }

        auto Solve(const Mazes::Grid<Kd, T>& _maze, const coord_t& _start, const coord_t& _end) {
            std::vector<coord_t> result;

            std::priority_queue<ASNode, std::vector<ASNode>, ASNodeCompare> openSet;

            ASNode start(_start, ASData(0.0f   , EuclideanDistance(_start, _end)));
            ASNode data1(_start, ASData(98.887f, 0.5f  ));
            ASNode data2(_start, ASData(17.5f  , 3.51f ));
            ASNode data3(_start, ASData(103.23f, 145.5f));
            ASNode data4(_start, ASData(27.05f , 12.2f ));

            openSet.push(start);
            openSet.push(data1);
            openSet.push(data2);
            openSet.push(data3);
            openSet.push(data4);

            const int size = openSet.size();
            for (int i = 0; i < size; i++) {
                std::cout << openSet.top().data.m_FScore << "\n";
                openSet.pop();
            }
        }

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
