#ifndef CHDR_ASTAR_HPP
#define CHDR_ASTAR_HPP

#include <cmath>

#include <Debug.hpp>

#include <functional>
#include <list>
#include <queue>
#include <unordered_set>

#include "base/ISolver.hpp"
#include "types/Heap.hpp"

namespace CHDR::Solvers {

    template<typename Tm, size_t Kd, typename Ts = float>
    class AStar : public ISolver<Tm> {

        static_assert(std::is_integral_v<Ts> || std::is_floating_point_v<Ts>, "Ts must be either an integral or floating point type");

    private:

        using coord_t = Coord<size_t, Kd>;

        struct ASNode;

        struct ASData {

            Ts m_GScore;
            Ts m_FScore;

            const ASNode* m_Parent;

            [[nodiscard]] constexpr ASData(const Ts& _gScore, const Ts& _hScore, const ASNode* const _parent) :
                m_GScore(_gScore),
                m_FScore(_gScore + _hScore),
                m_Parent(_parent) {}
        };

        struct ASNode {

            size_t  m_Coord;
            ASData  m_Data;

            [[nodiscard]] constexpr ASNode(const size_t& coord, const ASData& data) :
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

        void Solve(const Mazes::IMaze<Tm>& _maze) override {
            throw std::runtime_error("AStar::Solve(const Mazes::IMaze& _maze) Not implemented!");
        }

        auto Solve(const Mazes::Grid<Kd, Tm>& _maze, const coord_t& _start, const coord_t& _end, Ts (*_h)(const coord_t&, const coord_t&) = EuclideanDistance) const {

            std::vector<coord_t> result;

            const auto s = Utils::To1D(_start, _maze.Size());
            const auto e = Utils::To1D(_end,   _maze.Size());

            std::list<ASNode> buffer;

            std::unordered_set<size_t, std::function<const size_t&(const size_t&)>> closed(
                static_cast<size_t>(ceil(_h(_start, _end))),
                [](const size_t& _seed) constexpr -> const size_t& { return _seed; }
            );

            std::priority_queue<ASNode, std::vector<ASNode>, ASNodeCompare> openSet;
            openSet.emplace(ASNode(s, { static_cast<Ts>(0), _h(_start, _end), nullptr }));

            while (!openSet.empty()) {

                auto current = openSet.top();
                openSet.pop();

                if (current.m_Coord != e) {

                    /* SEARCH FOR SOLUTION */
                    closed.emplace(current.m_Coord);

                    buffer.emplace_back(std::move(current));
                    const auto& moved = buffer.back(); // 'current' no longer exists... it has been moved.

                    for (const auto neighbour : _maze.GetActiveNeighbours(moved.m_Coord)) {

                        const auto n = Utils::To1D(neighbour, _maze.Size());

                        // Check if node is not already visited:
                        auto search = closed.find(n);
                        if (search == closed.end()) {

                            // Add node to openSet.
                            openSet.emplace(ASNode(n, { moved.m_Data.m_GScore + static_cast<Ts>(1), _h(neighbour, _end), &moved }));
                        }
                    }
                }
                else {

                    /* SOLUTION REACHED */

                    // Clear the open set of all data:
                    std::priority_queue<ASNode, std::vector<ASNode>, ASNodeCompare>().swap(openSet);

                    // Recurse from end node to start node, inserting into a result buffer:
                    result.reserve(current.m_Data.m_GScore);

                    const auto* temp = &current;
                    while (temp != nullptr) {

                        result.push_back(Utils::ToND(temp->m_Coord, _maze.Size()));
                        temp = temp->m_Data.m_Parent;
                    }

                    // Reverse the buffer:
                    std::reverse(result.begin(), result.end());
                }
            }

            //PrintPath(result);

            return result;
        }

        static bool HasSolution(const Mazes::Grid<Kd, Tm>& _maze, const coord_t& _start, const coord_t& _end, size_t _capacity = 1U) {

	        bool result = false;

            const auto s = Utils::To1D(_start, _maze.Size());
            const auto e = Utils::To1D(_end,   _maze.Size());

            if (_maze.Contains(s) &&
                _maze.Contains(e) &&
                _maze.At(s).IsActive() &&
                _maze.At(e).IsActive()
            ) {
                std::queue<size_t> openSet;
                openSet.emplace(s);

                std::unordered_set<size_t> closedSet;
                closedSet.reserve(_capacity);
                closedSet.emplace(s);

                while (!openSet.empty()) {

                    for (size_t i = 0U; i < openSet.size(); ++i) {

                        const auto curr = openSet.front();
                        openSet.pop();

                        for (auto neighbour : _maze.GetActiveNeighbours(curr)) {

                            const auto n = Utils::To1D(neighbour, _maze.Size());

                            const auto [iter, inserted] = closedSet.emplace(n);
                            if (inserted) {

                                if (n == e) {
                                    result = true;

                                    goto NestedBreak;
                                }

                                openSet.emplace(n);
                            }
                        }
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

            Ts result(0);

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

            Ts result(0);

            for (size_t i = 0U; i < Kd; ++i) {
                result += abs(_B[i] - _A[i]);
            }

            return result;
        }

#pragma endregion Heuristics

    };
} // CHDR::Solvers

#endif //CHDR_ASTAR_HPP