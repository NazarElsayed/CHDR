#ifndef CHDR_ASTAR_HPP
#define CHDR_ASTAR_HPP

#include <cmath>

#include "base/ISolver.hpp"

#include <Debug.hpp>
#include <queue>

namespace CHDR::Solvers {

    template<typename T>
    class AStar : public ISolver<T> {

    private:

        template<size_t Kd>
        struct ASData {
            float m_GScore;
            float m_FScore;

            Coord<size_t, Kd> m_Parent;

            constexpr ASData(const float& _gScore, const float& _hScore, const Coord<size_t, Kd>& _parent = {}) :
                m_GScore(_gScore),
                m_FScore(_gScore + _hScore),
                m_Parent(_parent) {}
        };

        template<size_t Kd>
        struct NodeCompare {
            bool operator()(const ASData<Kd>& _a, const ASData<Kd>& _b) {
                return _a.m_FScore > _b.m_FScore;
            }
        };

    public:

        void Solve(const Mazes::IMaze<T>& _maze) override {
            throw std::runtime_error("AStar::Solve(const Mazes::IMaze& _maze) Not implemented!");
        }

        template<size_t Kd>
        auto Solve(const Mazes::Grid<Kd, T>& _maze, const Coord<size_t, Kd>& _start, const Coord<size_t, Kd>& _end) {
            using data_t  = ASData<Kd>;
            using coord_t = Coord<size_t, Kd>;

            std::vector<coord_t> result;

            std::priority_queue<data_t, std::vector<data_t>, NodeCompare<Kd>> openSet;

            data_t startData(0.0f, EuclideanDistance(_start, _end));
            data_t data1(17.0f, 10.2f);
            data_t data2(53.0f, 16.4f);
            data_t data3(34.0f, 11.8f);
            data_t data4(21.0f, 53.2f);

            openSet.push(startData);
            openSet.push(data1);
            openSet.push(data2);
            openSet.push(data3);
            openSet.push(data4);

            std::cout << std::endl;
            std::cout << "Priority queue data" << "\n";

            const int size = openSet.size();
            for (int i = 0; i < size; i++) {
                std::cout << openSet.top().m_FScore << "\n";
                openSet.pop();
            }

            std::cout << std::endl;

            //std::unordered_map<coord_t, data_t> openSet;
            //openQueue.push(_start);

            //std::pair<coord_t, data_t> start = std::make_pair(_start, startData);

            //openSet.insert(start);
        }

        template<size_t Kd>
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
        template<size_t Kd>
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
        template<size_t Kd>
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
        template<size_t Kd>
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
