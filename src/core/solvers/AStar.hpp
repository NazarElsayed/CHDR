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

    template<typename Tm, size_t Kd, typename Ts = float>
    class AStar : public ISolver<Tm> {

        static_assert(std::is_integral_v<Ts> || std::is_floating_point_v<Ts>, "Ts must be either an integral or floating point type");

    private:

        using coord_t = Coord<size_t, Kd>;

        struct ASData {

            Ts m_GScore;
            Ts m_FScore;

            coord_t m_Parent;

            [[nodiscard]] constexpr ASData(const Ts& _gScore, const Ts& _hScore, const coord_t& _parent = {}) :
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

                std::hash<Tm> hashing_func;

                for (size_t i = 0U; i < Kd; ++i) {
                    result ^= hashing_func(_node.m_Coord[i]) << i % (sizeof(size_t) * 8U);
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

        void Solve(const Mazes::IMaze<Tm>& _maze) override {
            throw std::runtime_error("AStar::Solve(const Mazes::IMaze& _maze) Not implemented!");
        }

        auto Solve(const Mazes::Grid<Kd, Tm>& _maze, const coord_t& _start, const coord_t& _end, float (*_h)(const coord_t&, const coord_t&) = ManhattanDistance) const {

            std::vector<coord_t> result;

            std::unordered_map<coord_t, ASNode> closedSet;
            std::priority_queue<ASNode, std::vector<ASNode>, ASNodeCompare> openSet;

            openSet.emplace(ASNode(_start, ASData(0, _h(_start, _end))));

            bool complete(false);
            while (!complete) {

                const auto current = openSet.top();
                openSet.pop();

                if (current.m_Coord == _end) {

                    result.push_back(current.m_Coord);

                    auto node = closedSet.at(current.m_Data.m_Parent);
                    while (node.m_Coord != _start) {
                        result.push_back(node.m_Coord);
                        node = closedSet.at(node.m_Data.m_Parent);
                    }

                    std::reverse(result.begin(), result.end());

                    complete = true;
                }
                else {

                    for (const auto neighbour : _maze.GetActiveNeighbours(current.m_Coord)) {

                        // Check node already exists in collections:
                        auto search = closedSet.find(neighbour);
                        if (search == closedSet.end()) {

                            // Add node to openSet.
                            openSet.emplace(ASNode(neighbour, ASData(current.m_Data.m_GScore + 1, _h(neighbour, _end), current.m_Coord)));
                        }
                    }

                    closedSet.emplace(current.m_Coord ,current);
                }

                if (openSet.empty()) {
                    complete = true;
                }
            }

            //PrintPath(result);

            return result;
        }


    static bool HasSolution(const Mazes::Grid<Kd, Tm>& _maze, const coord_t& _start, const coord_t& _end) {

	        bool result = false;

            if (_maze.Contains(_start) &&
                _maze.Contains(_end  ) &&
                _maze.At(_start).IsActive() &&
                _maze.At(_end  ).IsActive()
            ) {
                    std::vector<coord_t> openSet;
                    openSet.emplace_back(_start);

                    std::unordered_set<coord_t> closedSet;
                    closedSet.emplace(_start);

                    while (!openSet.empty()) {

                        for (size_t i = 0U; i < openSet.size(); ++i) {

                            const auto& curr = openSet[i];

                            for (auto neighbour : _maze.GetActiveNeighbours(curr)) {

                                auto search = closedSet.find(neighbour);
                                if (search == closedSet.end()) {

                                    if (neighbour == _end) {
                                        result = true;

                                        goto NestedBreak;
                                    }

                                    openSet.emplace_back(neighbour);
                                    closedSet.emplace(neighbour);
                                }
                            }

                            openSet.erase(openSet.begin() + i);
                        }
                    }
                }

NestedBreak:
                return result;
        }

        constexpr void PrintPath(std::vector<coord_t>& _path) const {

            for (const auto& coord : _path) {

                std::ostringstream oss;
                oss << "(";

                for (size_t i = 0U; i < Kd; ++i) {
                    oss << coord[i] << (i < Kd - 1U ? ", " : "");
                }

                oss << ")";

                Debug::Log(oss.str(), Info);
            }
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

            Ts result = 0;

            for (size_t i = 0U; i < Kd; ++i) {
                const auto val = _B[i] - _A[i];
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

            Ts result = 0;

            for (size_t i = 0U; i < Kd; ++i) {
                result += abs(_B[i] - _A[i]);
            }

            return result;
        }

#pragma endregion Heuristics

    };
} // CHDR::Solvers

#endif //CHDR_ASTAR_HPP