#ifndef CHDR_ASTAR_HPP
#define CHDR_ASTAR_HPP

#include <cmath>

#include "base/ISolver.hpp"

#include <Debug.hpp>
#include <queue>
#include <unordered_set>

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

            coord_t m_Coord;
            ASData  m_Data;

            [[nodiscard]] constexpr ASNode(const coord_t& coord, const ASData& data) :
                m_Coord(coord),
                m_Data(data) {}

            [[nodiscard]] constexpr bool operator == (const ASNode& _node) const {
                return m_Coord == _node.m_Coord;
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

            std::unordered_set<coord_t> closedSet;

            std::priority_queue<ASNode, std::vector<ASNode>, ASNodeCompare> openSet;
            openSet.emplace(ASNode(_start, { 0.0F, _h(_start, _end) }));

            bool complete(false);
            while (!complete) {

                const auto curr = std::move(openSet.top());
                openSet.pop();

                if (curr.m_Coord == _end) {
                    result.push_back(curr.m_Coord);

                    complete = true;
                }
                else {

                    // Grab adjacent neighbours in each dimention. Does not include diagonals.
                    for (size_t i = 0U; i < Kd; i++) {

                        {
                            auto leftCoord = curr.m_Coord;
                            --leftCoord[i];

                            openSet.emplace(
                                ASNode(
                                    leftCoord,
                                    {
                                        _h(curr.m_Coord, leftCoord),
                                        _h(leftCoord, _end),
                                        curr.m_Coord
                                    }
                                )
                            );
                        }

                        {
                            auto rightCoord = curr.m_Coord;
                            ++rightCoord[i];

                            openSet.emplace(
                                ASNode(
                                    rightCoord,
                                    {
                                        _h(curr.m_Coord, rightCoord),
                                        _h(rightCoord, _end),
                                        curr.m_Coord
                                    }
                                )
                            );
                        }
                    }
                }

                if (openSet.empty()) {
                    complete = true;
                }
            }

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