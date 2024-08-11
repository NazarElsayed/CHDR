#ifndef CHDR_ASTAR_HPP
#define CHDR_ASTAR_HPP

#include <cmath>

#include <Debug.hpp>

#include <functional>
#include <list>
#include <queue>

#include "base/ISolver.hpp"
#include "types/Heap.hpp"
#include "types/DenseExistenceSet.hpp"
#include "types/HeavyNode.hpp"

#include "../utils/Heuristics.hpp"

namespace CHDR::Solvers {

    template<typename Tm, const size_t Kd, typename Ts>
    class AStar final : public ISolver<Tm> {

        static_assert(std::is_integral_v<Ts> || std::is_floating_point_v<Ts>,
                      "Ts must be either an integral or floating point type");

    private:

        using coord_t = Coord<size_t, Kd>;

        /*struct ASNode final : IHeapItem {

            size_t m_Coord;

            Ts m_GScore;
            Ts m_FScore;

            const ASNode* m_Parent;

            [[nodiscard]] constexpr ASNode(const size_t& _coord, const Ts& _gScore, const Ts& _hScore, const ASNode* const _parent) : IHeapItem(),
                m_Coord(_coord),
                m_GScore(_gScore),
                m_FScore(_gScore + _hScore),
                m_Parent(_parent) {}

            [[nodiscard]] constexpr bool operator ==(const ASNode& _node) const { return m_Coord == _node.m_Coord; }
        };*/

        /*struct ASNodeCompare {
            [[nodiscard]] constexpr bool operator ()(const ASNode& _a, const ASNode& _b) const {
                return _a.m_FScore > _b.m_FScore;
            }
        };*/

        /*struct ASHeavyNode : IHeapItem {

            size_t m_Position;
            NodeData m_Data;

            [[nodiscard]] constexpr bool operator ()(const NodeData& _a, const NodeData& _b) const {
                return _a.m_FScore > _b.m_FScore;
            }
        };*/

        struct NodeDataCompare {
            [[nodiscard]] constexpr bool operator ()(const NodeData& _a, const NodeData& _b) const {
                return _a.m_FScore > _b.m_FScore;
            }
        };

    public:

        void Solve(const Mazes::IMaze<Tm>& _maze) override {
            throw std::runtime_error("AStar::Solve(const Mazes::IMaze& _maze) Not implemented!");
        }

        /*
        auto Solve(const Mazes::Grid<Kd, Tm>& _maze, const coord_t& _start, const coord_t& _end,
                   Ts (*                           _h)(const coord_t&, const coord_t&)) const {
            std::vector<coord_t> result;

            const auto s = Utils::To1D(_start, _maze.Size());
            const auto e = Utils::To1D(_end, _maze.Size());

            std::list<ASNode> buffer;

            DenseExistenceSet closedSet(std::max(s, e));

            Heap<ASNode, ASNodeCompare> openSet;

            openSet.Emplace({s, static_cast<Ts>(0), _h(_start, _end), nullptr});

            while (!openSet.Empty()) {
                buffer.emplace_back(std::move(openSet.Top()));
                const auto current = openSet.Top();
                openSet.RemoveFirst();

                if (current.m_Coord != e) {
                    /* SEARCH FOR SOLUTION #1#

                    if (closedSet.Capacity() <= current.m_Coord) {
                        closedSet.Reserve(std::min(closedSet.Capacity() * 2U, Utils::Product<size_t>(_maze.Size())));
                    }
                    closedSet.Add(current.m_Coord);

                    for (const auto    neighbour : _maze.GetNeighbours(current.m_Coord)) {
                        if (const auto [nActive, nValue] = neighbour; nActive) {
                            const auto n = Utils::To1D(nValue, _maze.Size());

                            // Check if node is not already visited:
                            if (!closedSet.Contains(n)) {
                                // Add node to openSet.
                                openSet.Emplace({n, current.m_GScore + static_cast<Ts>(1), _h(nValue, _end), &current});
                            }
                        }
                    }
                }
                else {
                    /* SOLUTION REACHED #1#

                    // Free data which is no longer relevant:
                    openSet.Clear();
                    closedSet.Clear();

                    // Recurse from end node to start node, inserting into a result buffer:
                    result.reserve(current.m_GScore);

                    for (const auto* temp = &current; temp->m_Parent != nullptr; temp = temp->m_Parent) {
                        result.emplace_back(Utils::ToND(temp->m_Coord, _maze.Size()));
                    }

                    buffer.clear();

                    // Reverse the result:
                    std::reverse(result.begin(), result.end());

                    break;
                }
            }

            return result;
        }*/

        auto Solve(Mazes::HeavyGrid<Kd, Tm>& _maze, const coord_t& _start, const coord_t& _end, Ts (*_h)(const coord_t&, const coord_t&)) const {

            std::vector<coord_t> result;

            const auto& mazeSize = _maze.Size();

            const auto s = Utils::To1D(_start, mazeSize);
            const auto e = Utils::To1D(_end, mazeSize);

            //DenseExistenceSet closedSet(std::max(s, e));
            Heap<NodeData, NodeDataCompare> openSet;

            auto& start = *_maze.GetNode(s).Data();
            start.m_Position = s;
            start.m_GScore = 0;
            start.m_FScore = _h(_start, _end);
            start.m_Parent = s;

            openSet.Add(start);

            while (!openSet.Empty()) {

                auto& current = *_maze.GetNode(openSet.Top().m_Position).Data();

                if (current.m_Closed) {
                    Debug::Log("Here");
                }

                openSet.RemoveFirst();

                if (current.m_Position != e) {

                    current.m_Closed = true;

                    /* SEARCH FOR SOLUTION */
                    for (const auto neighbour : _maze.GetNeighbours(Utils::ToND(current.m_Position, _maze.Size()))) {
                        if (const auto [nActive, nValue] = neighbour; nActive) {
                            const auto n = Utils::To1D(nValue, _maze.Size());

                            auto& neighbourData = *_maze.GetNode(n).Data();

                            // Check if node is not already visited:
                            if (!neighbourData.m_Closed) {

                                Ts gScore = current.m_GScore + 1;

                                //if (gScore < neighbourData.m_GScore) {

                                    neighbourData.m_Position = n;
                                    neighbourData.m_GScore = gScore;
                                    neighbourData.m_FScore = gScore + _h(nValue, _end);
                                    neighbourData.m_Parent = current.m_Position;
                                    openSet.Add(neighbourData);

                                    /*if (openSet.Contains(neighbourData)) {
                                        openSet.Update(neighbourData);
                                    } else {
                                        openSet.Add(neighbourData);
                                    }*/
                                //}
                            }
                        }
                    }
                }
                else {
                    /* SOLUTION REACHED */

                    // Free data which is no longer relevant:
                    openSet.Clear();

                    // Recurse from end node to start node, inserting into a result buffer:
                    result.reserve(current.m_GScore);
                    for (auto temp = current; temp.m_Parent != temp.m_Position; temp = *_maze.GetNode(temp.m_Parent).Data()) {
                        result.emplace_back(Utils::ToND(temp.m_Position, _maze.Size()));
                    }

                    // Reverse the result:
                    std::reverse(result.begin(), result.end());

                    break;
                }
            }

            return result;
        }

        static bool HasSolution(const Mazes::HeavyGrid<Kd, Tm>& _maze, const coord_t& _start, const coord_t& _end, size_t _capacity = 1U) {

            bool result = false;

            const auto s = Utils::To1D(_start, _maze.Size());
            const auto e = Utils::To1D(_end, _maze.Size());

            if (_maze.Contains(s) &&
                _maze.Contains(e) &&
                _maze.At(s).IsActive() &&
                _maze.At(e).IsActive()
            ) {
                std::queue<size_t> openSet;
                openSet.emplace(s);

                DenseExistenceSet closedSet({s}, std::max(s, e));

                while (!openSet.empty()) {
                    for (size_t i = 0U; i < openSet.size(); ++i) {
                        const auto curr = openSet.front();
                        openSet.pop();

                        for (const auto    neighbour : _maze.GetNeighbours(curr)) {
                            if (const auto [nActive, nValue] = neighbour; nActive) {
                                const auto n = Utils::To1D(nValue, _maze.Size());

                                if (!closedSet.Contains(n)) {
                                    if (closedSet.Capacity() <= n) {
                                        closedSet.Reserve(std::min(closedSet.Capacity() * 2U,
                                                                   Utils::Product<size_t>(_maze.Size())));
                                    }
                                    closedSet.Add(n);

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

    };
} // CHDR::Solvers

#endif //CHDR_ASTAR_HPP
