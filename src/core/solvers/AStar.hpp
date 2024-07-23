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

            Coord<size_t, Kd> m_Parent;

            constexpr ASData(const float& _gScore, const float& _hScore, const coord_t& _parent = {}) :
                m_GScore(_gScore),
                m_FScore(_gScore + _hScore),
                m_Parent(_parent) {}
        };

        struct NodeCompare {
            bool operator()(const std::pair<coord_t, ASData>& _a, const std::pair<coord_t, ASData>& _b) {
                return _a.second.m_FScore > _b.second.m_FScore;
            }
        };

    public:

        void Solve(const Mazes::IMaze<T>& _maze) override {
            throw std::runtime_error("AStar::Solve(const Mazes::IMaze& _maze) Not implemented!");
        }

        auto Solve(const Mazes::Grid<Kd, T>& _maze, const Coord<size_t, Kd>& _start, const Coord<size_t, Kd>& _end) {

            using node_t = std::pair<coord_t, ASData>;

            std::vector<coord_t> result;

            std::priority_queue<node_t, std::vector<node_t>, NodeCompare> openSet;

            ASData startData(0.0f, EuclideanDistance(_start, _end));
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
