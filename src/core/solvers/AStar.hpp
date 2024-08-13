#ifndef CHDR_ASTAR_HPP
#define CHDR_ASTAR_HPP

#include <cmath>

#include <Debug.hpp>

#include <forward_list>
#include <functional>
#include <queue>
#include <unordered_set>

#include "base/ISolver.hpp"
#include "types/Heap.hpp"
#include "types/DenseExistenceSet.hpp"

#include "../utils/Heuristics.hpp"

namespace CHDR::Solvers {

    template<typename Tm, const size_t Kd, typename Ts>
    class AStar final : public ISolver<Tm> {

        static_assert(std::is_integral_v<Ts> || std::is_floating_point_v<Ts>, "Ts must be either an integral or floating point type");

    private:

        using coord_t = Coord<size_t, Kd>;

        struct ASNode final : IHeapItem {

            size_t m_Coord;

            Ts m_GScore;
            Ts m_FScore;

            const ASNode* m_Parent;

            [[nodiscard]] constexpr ASNode(const size_t& _coord, const Ts& _gScore, const Ts& _hScore, const ASNode* const _parent) : IHeapItem(),
                m_Coord (_coord),
                m_GScore(_gScore),
                m_FScore(_gScore + _hScore),
                m_Parent(_parent) {}

            ~ASNode() {

                if (m_Parent == nullptr) {
                    delete m_Parent;

                    m_Parent = nullptr;
                }
            };

            [[nodiscard]] constexpr bool operator == (const ASNode& _node) const { return m_Coord == _node.m_Coord; }

            struct Max {
                [[nodiscard]] constexpr bool operator () (const ASNode& _a, const ASNode& _b) const { return _a.m_FScore > _b.m_FScore; }
            };
        };

        struct SMASNode final : IHeapItem {

            bool m_Expanded;

            size_t m_Depth;
            size_t m_Coord;

            Ts m_GScore;
            Ts m_FScore;

            SMASNode* m_Parent;

            std::vector<SMASNode> m_Successors;

            std::unordered_set<size_t, size_t> m_ForgottenFCosts;

            [[nodiscard]] constexpr SMASNode(const size_t& _depth, const size_t& _coord, const Ts& _gScore, const Ts& _hScore, SMASNode* const _parent) : IHeapItem(),
                m_Expanded(false),
                m_Depth (_depth),
                m_Coord (_coord),
                m_GScore(_gScore),
                m_FScore(_gScore + _hScore),
                m_Parent(_parent),
                m_Successors()
            {
                if (m_Parent != nullptr) {
                    m_Parent->m_Successors.emplace_back(*this);
                }
            }

            [[nodiscard]] constexpr bool operator == (const SMASNode& _node) const { return m_Coord == _node.m_Coord; }

            struct Max {
                [[nodiscard]] constexpr bool operator () (const SMASNode& _a, const SMASNode& _b) const { return _a.m_FScore > _b.m_FScore; }
            };
        };

    public:

        void Solve(const Mazes::IMaze<Tm>& _maze) override {
            throw std::runtime_error("AStar::Solve(const Mazes::IMaze& _maze) Not implemented!");
        }

        auto Solve(const Mazes::Grid<Kd, Tm>& _maze, const coord_t& _start, const coord_t& _end, Ts (*_h)(const coord_t&, const coord_t&)) const {

            std::vector<coord_t> result;

            const auto s = Utils::To1D(_start, _maze.Size());
            const auto e = Utils::To1D(_end,   _maze.Size());

            DenseExistenceSet closedSet(std::max(s, e));

            Heap<ASNode, typename ASNode::Max> openSet;
            openSet.Emplace({ s, static_cast<Ts>(0), _h(_start, _end), nullptr });

            while (!openSet.Empty()) {

                const ASNode current = openSet.Top();
                openSet.RemoveFirst();

                if (current.m_Coord != e) {

                    /* SEARCH FOR SOLUTION */

                    if (closedSet.Capacity() <= current.m_Coord) {
                        closedSet.Reserve(std::min(closedSet.Capacity() * 2U, Utils::Product<size_t>(_maze.Size())));
                    }
                    closedSet.Add(current.m_Coord);

                    for (const auto neighbour : _maze.GetNeighbours(current.m_Coord)) {

                        if (const auto [nActive, nValue] = neighbour; nActive) {

                            const auto n = Utils::To1D(nValue, _maze.Size());

                            // Check if node is not already visited:
                            if (!closedSet.Contains(n)) {

                                // Create room for 'current' in unmanaged memory:
                                void* memoryBlock = std::malloc(sizeof(ASNode));
                                if (!memoryBlock) { throw std::bad_alloc(); }

                                // Create a parent node and transfer ownership of 'current' to it. Note: 'current' is now moved!
                                openSet.Emplace({ n, current.m_GScore + static_cast<Ts>(1), _h(nValue, _end), new (memoryBlock) ASNode(std::move(current)) });
                            }
                        }
                    }
                }
                else {

                    /* SOLUTION REACHED */

                    // Free data which is no longer relevant:
                      openSet.Clear();
                    closedSet.Clear();

                    // Recurse from end node to start node, inserting into a result buffer:
                    result.reserve(current.m_GScore);

                    for (const auto* temp = &current; temp->m_Parent != nullptr; temp = temp->m_Parent) {
                        result.emplace_back(Utils::ToND(temp->m_Coord, _maze.Size()));
                    }

                    // Reverse the result:
                    std::reverse(result.begin(), result.end());

                    break;
                }
            }

            return result;
        }

        auto Solve(const Mazes::Grid<Kd, Tm>& _maze, const coord_t& _start, const coord_t& _end, Ts (*_h)(const coord_t&, const coord_t&), const size_t& _memoryLimit) const {

            /** @see: https://easychair.org/publications/open/TL2M */

            std::vector<coord_t> result;

            const auto s = Utils::To1D(_start, _maze.Size());
            const auto e = Utils::To1D(_end,   _maze.Size());

            Heap<SMASNode, typename SMASNode::Max> openSet;
            openSet.Emplace({
                0U,                 // Depth
                s,                  // Coordinate
                static_cast<Ts>(0), // G-Score
                _h(_start, _end),   // F-Score
                nullptr             // Parent
            });

            size_t u = 1U; // counter for nodes in memory

            // Main loop
            while (!openSet.empty()) {

                SMASNode b = openSet.Top(); // Node with smallest f-cost in O
                openSet.RemoveFirst();

                // If node b is goal
                if (b.m_Coord != e) {

                    if (b.m_FScore != std::numeric_limits<Ts>::infinity()) {

                        if (!b.m_Expanded) {
                            b.m_Expanded = true;

                            // Expand b and assign its successors to N
                            auto neighbours = _maze.GetNeighbours(b.m_Coord);

                            for (auto& neighbour : neighbours) {

                                if (const auto [nActive, nValue] = neighbour; nActive) {

                                    if (nValue != b.m_Coord) {

                                        b.m_Successors.emplace_back({
                                            b.m_Depth + 1U,     // Depth
                                            nValue,             // Coordinate
                                            b.m_GScore + 1U,    // G-Score
                                            _h(nValue, _end),   // F-Score
                                            b                   // Parent
                                        });
                                    }
                                }
                            }

                        }

                        std::vector<SMASNode> N = b.m_Successors; // Set N as successors of b

                        for (SMASNode& n : N) {

                            const auto nIdx = Utils::ToND(n, _maze.Size());

                            if (b.m_ForgottenFCosts.find(n.m_Coord) != b.m_ForgottenFCosts.end()) { /* condition to check if s(n) is in forgotten f-cost table of b*/
                                n.m_FScore = b.m_ForgottenFCosts(n); // f-value of s(n) in forgotten f-cost table of node b
                                b.m_ForgottenFCosts.erase(n.m_Coord); // Remove s(n) from forgotten f-cost table of node b.
                            }
                            else if (nIdx != e && (n.m_Successors.empty() || n.m_Depth >= _memoryLimit - 1U)) {
                                n.m_FScore = std::numeric_limits<Ts>::infinity();
                            }
                            else {
                                // Update properties of n according to the pseudocode
                                n.m_FScore = std::max(b.m_FScore, n.m_GScore + _h(n.m_Coord, _end));
                            }

                            // Add n to O
                            openSet.Emplace(n);
                            u++;
                        }

                        while (u > _memoryLimit) {
                            cull_worst_leaf(openSet, u);
                        }
                    }
                    else {
                        break; // Return goal not found
                    }
                }
                else {

                    /* SOLUTION REACHED */

                    // Free data which is no longer relevant:
                    openSet.Clear();

                    // Recurse from end node to start node, inserting into a result buffer:
                    result.reserve(b.m_GScore);

                    for (const auto* temp = &b; temp->m_Parent != nullptr; temp = temp->m_Parent) {
                        result.emplace_back(Utils::ToND(temp->m_Coord, _maze.Size()));
                    }

                    // Reverse the result:
                    std::reverse(result.begin(), result.end());

                    break;
                }
            }

            return result;
        }

        void cull_worst_leaf(Heap<SMASNode, typename ASNode::Max>& _openSet, size_t& _memoryLimit) {

            const SMASNode w = safe_culling_heuristic(_openSet);

            auto p = w.m_Parent; // parent node of w

            // Remove w from the successor list of p
            for (size_t i = 0U; i < p->m_Successors.size(); ++i) {

                // Code to remove w from the successor list of p goes here
                if (p->m_Successors[i] == w) {
                    p->m_Successors.erase(p->m_Successors.begin() + i);

                    break;
                }
            }

            // Add s(w) to forgotten f-cost table of p, with value of f (w)
            p->m_ForgottenFCosts.insert_or_assign(p, w.m_FScore);

            // f (p) ← min of forgotten f-costs of p
            for (auto& [state, cost] : p->m_ForgottenFCosts) {
                p->m_FScore = std::min(p->m_FScore, cost);
            }

            // if p is not in _openSet then
            if (!_openSet.Contains(p)) {
                _openSet.push_back(p); // Add p to _openSet
            }

            _memoryLimit--;
        }

        auto safe_culling_heuristic(Heap<SMASNode, typename ASNode::Max>& _openSet) {

            SMASNode w; // Worst leaf according to c(n) in _openSet

            const SMASNode b = _openSet.Top(); // Best node according to f(n) in _openSet

            if (w == b) {

                // Code to find second worst leaf according to c(n) goes here
                // Assign the second worst leaf to w

                w = _openSet.Back();

                for (size_t i = _openSet.Size() / 2U; i < _openSet.Size(); ++i) {

                    if (SMASNode::Max(_openSet[i], w.val)) {
                        w = _openSet[i];
                    }
                }

                _openSet.Remove(w);
            }
            else {
                w = _openSet.Back();
                w.RemoveLast();
            }

            return w;
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

                DenseExistenceSet closedSet({ s }, std::max(s, e));

                while (!openSet.empty()) {

                    for (size_t i = 0U; i < openSet.size(); ++i) {

                        const auto curr = openSet.front();
                        openSet.pop();

                        if (curr == e) {
                            result = true;

                            goto NestedBreak;
                        }

                        for (const auto neighbour : _maze.GetNeighbours(curr)) {

                            if (const auto [nActive, nValue] = neighbour; nActive) {

                                const auto n = Utils::To1D(nValue, _maze.Size());

                                if (!closedSet.Contains(n)) {

                                    if (closedSet.Capacity() <= n) {
                                        closedSet.Reserve(std::min(closedSet.Capacity() * 2U, Utils::Product<size_t>(_maze.Size())));
                                    }
                                    closedSet.Add(n);

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