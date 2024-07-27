#ifndef CHDR_ASTAR_HPP
#define CHDR_ASTAR_HPP

#include <cmath>

#include "base/ISolver.hpp"

#include <Debug.hpp>
#include <queue>
#include <unordered_set>

#include <functional>

#include "types/Heap.hpp"

namespace CHDR::Solvers {

    template<typename T, size_t Kd>
    class AStar : public ISolver<T> {

    private:

        using coord_t = Coord<size_t, Kd>;

        struct ASData {

            float m_GScore;
            float m_FScore;

            coord_t m_Parent;

            [[nodiscard]] constexpr ASData(const float& _gScore, const float& _hScore, const coord_t& _parent = {}) :
                m_GScore(_gScore),
                m_FScore(_gScore + _hScore),
                m_Parent(_parent) {}

        };

        struct ASNode {

            size_t m_HeapIndex;

            coord_t m_Coord;
            ASData  m_Data;

            [[nodiscard]] constexpr ASNode(const coord_t& coord, const ASData& data) :
                m_HeapIndex(0U),
                m_Coord(coord),
                m_Data(data) {}

            [[nodiscard]] constexpr bool operator == (const ASNode& _node) const {
                return m_Coord == _node.m_Coord;
            }
        };

        struct ASNodeHash {

            size_t operator()(const ASNode& _node) const {

                size_t result = 0U;

                std::hash<T> hashing_func;

                for (size_t i = 0U; i < Kd; ++i) {
                    result ^= hashing_func(_node.m_Coord[i]) << i % (sizeof(size_t)*8);
                }

                return result;
            }
        };

        struct ASNodeCompare {

            [[nodiscard]] constexpr bool operator () (const ASNode& _a, const ASNode& _b) const {
                return _a.m_Data.m_FScore > _b.m_Data.m_FScore;
            }
        };

    public:

        void Solve(const Mazes::IMaze<T>& _maze) override {
            throw std::runtime_error("AStar::Solve(const Mazes::IMaze& _maze) Not implemented!");
        }

        auto Solve(const Mazes::Grid<Kd, T>& _maze, const coord_t& _start, const coord_t& _end) const {

            std::vector<coord_t> result;

            std::unordered_set<ASNode, ASNodeHash> closedSet;
            Heap<ASNode, ASNodeCompare> openSet;

            ASNode start(_start, ASData(0.0f, _h(_start, _end)));

            openSet.Push(start);

            bool complete(false);
            while (!complete) {

                const auto current = openSet.Top();
                openSet.Pop();

                if (current.m_Coord == _end) {
                    result.push_back(current.m_Coord);

                    complete = true;
                }
                else {

                    for (const auto& neighbour : _maze.GetActiveNeighbours(current.m_Coord)) {

                        ASNode node (neighbour, ASData(current.m_Data.m_GScore + 1, _h(neighbour, _end), current.m_Coord));

                        //Check node already exists in collections
                        auto search = closedSet.find(node);
                        if (search != closedSet.end()) {
                            //Update node in collections

                            auto& existingNode = *search;
                            if (node.m_Data.m_FScore < existingNode.m_Data.m_FScore) {

                                //Update item in openSet and resort value.
                                openSet.Push(node);
                            }
                        }
                        else {
                            //Add node to openSet
                            openSet.Push(node);
                        }
                    }

                    closedSet.insert(current);
                }

                if (openSet.Empty()) {
                    complete = true;
                }
            }

            std::cout << result[0][0] << ", " << result[0][1] << std::endl;

            return result;
        }

        [[nodiscard]] static constexpr auto _h(const coord_t& _A, const coord_t& _B) {
            return EuclideanDistance(_A, _B);
            //return SqrEuclideanDistance(_A, _B);
            //return ManhattanDistance(_A, _B);
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
        [[nodiscard]] static constexpr auto EuclideanDistance(const coord_t& _A, const coord_t& _B) {
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
        [[nodiscard]] static constexpr auto SqrEuclideanDistance(const coord_t& _A, const coord_t& _B) {

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
        [[nodiscard]] static constexpr auto ManhattanDistance(const coord_t& _A, const coord_t& _B) {

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