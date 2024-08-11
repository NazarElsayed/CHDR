#ifndef CHDR_ASTAR_HPP
#define CHDR_ASTAR_HPP

#include <cmath>

#include <Debug.hpp>

#include <forward_list>
#include <functional>
#include <queue>

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

            size_t m_Coord;

            Ts m_GScore;
            Ts m_FScore;

            SMASNode* m_Parent;

            std::vector<SMASNode> m_Successors;

            [[nodiscard]] constexpr SMASNode(const size_t& _coord, const Ts& _gScore, const Ts& _hScore, SMASNode* const _parent) : IHeapItem(),
                m_Coord (_coord),
                m_GScore(_gScore),
                m_FScore(_gScore + _hScore),
                m_Parent(_parent),
                m_Successors()
            {
                if (m_Parent != nullptr) {
                    m_Parent->m_Children.emplace_back(*this);
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

        // auto Solve(const Mazes::Grid<Kd, Tm>& _maze, const coord_t& _start, const coord_t& _end, Ts (*_h)(const coord_t&, const coord_t&), const size_t& _m) const {
        //
        //     /** @see: https://easychair.org/publications/open/TL2M */
        //
        //     std::vector<coord_t> result;
        //
        //     const auto s = Utils::To1D(_start, _maze.Size());
        //     const auto e = Utils::To1D(_end,   _maze.Size());
        //
        //     Heap<SMASNode, typename ASNode::Max> openSet;
        //     openSet.Emplace({ s, static_cast<Ts>(0), _h(_start, _end), nullptr });
        //
        //     size_t u = 1U; // counter for nodes in memory
        //
        //     // Main loop
        //     while (!openSet.empty()) {
        //
        //         const SMASNode b = openSet.Top(); // Node with smallest f-cost in O
        //         openSet.RemoveFirst();
        //
        //         // If node b is goal
        //         if (b.m_Coord != e) {
        //
        //             if (b.m_FScore != std::numeric_limits<Ts>::infinity()) {
        //
        //                 std::vector<SMASNode> N; // Set N as successors of b
        //
        //                 if (/* condition for whether b has been expanded */) {
        //                     // Assign forgotten successors of b to N
        //                 }
        //                 else {
        //                     // Expand b and assign its successors to N
        //                     N = b.m_Successors;
        //                 }
        //
        //                 for (SMASNode& n : N) {
        //
        //                     const auto nIdx = Utils::ToND(n, _maze.Size());
        //
        //                     if (/* condition to check if s(n) is in forgotten f-cost table of b*/) {
        //                         //n.m_FScore = f-value of s(n) in forgotten f-cost table of node b
        //                         // Remove s(n) from forgotten f-cost table of node b.
        //                     }
        //                     else if (nIdx != e && (n.m_Successors.empty() || /* d(n) >= M - 1*/)) {
        //                         n.m_FScore = std::numeric_limits<Ts>::infinity();
        //                     }
        //                     else {
        //                         // Update properties of n according to the pseudocode
        //                         n.m_FScore = std::max(b.m_FScore, n.m_GScore + _h(n.m_Coord, _end));
        //                     }
        //
        //                     // Add n to O
        //                     openSet.Emplace(n);
        //                     u++;
        //                 }
        //
        //                 while (u > _m) {
        //                     cull_worst_leaf(openSet, u);
        //                 }
        //             }
        //             else {
        //                 break; // Return goal not found
        //             }
        //         }
        //         else {
        //
        //             /* SOLUTION REACHED */
        //
        //             // Free data which is no longer relevant:
        //             openSet.Clear();
        //
        //             // Recurse from end node to start node, inserting into a result buffer:
        //             result.reserve(b.m_GScore);
        //
        //             for (const auto* temp = &b; temp->m_Parent != nullptr; temp = temp->m_Parent) {
        //                 result.emplace_back(Utils::ToND(temp->m_Coord, _maze.Size()));
        //             }
        //
        //             // Reverse the result:
        //             std::reverse(result.begin(), result.end());
        //
        //             break;
        //         }
        //     }
        //
        //     return result;
        // }
        //
        // // Algorithm 2
        // void cull_worst_leaf(Heap<SMASNode, typename ASNode::Max>& O, size_t& u) {
        //
        //     const SMASNode w = safe_culling_heuristic(O);
        //
        //     auto p = w.m_Parent; // parent node of w
        //
        //     // Remove w from the successor list of p
        //     for (size_t i = 0U; i < p->m_Successors.size(); ++i) {
        //
        //         // Code to remove w from the successor list of p goes here
        //         if (p->m_Successors[i] == w) {
        //             p->m_Successors.erase(p->m_Successors.begin() + i);
        //
        //             break;
        //         }
        //     }
        //
        //     // Add s(w) to forgotten f-cost table of p, with value of f (w)
        //     // Code to add s(w) to forgotten f-cost table of p, with value of f (w) goes here
        //
        //     // f (p) ← min of forgotten f-costs of p
        //     // Code to calculate the min of forgotten f-costs of p goes here
        //
        //     // if p is not in O then
        //     if (!O.Contains(p)) {
        //         O.push_back(p); // Add p to O
        //     }
        //
        //     u--;
        // }
        //
        // auto safe_culling_heuristic(Heap<SMASNode, typename ASNode::Max>& O) {
        //
        //     SMASNode w; // Worst leaf according to c(n) in O
        //
        //     const SMASNode b = O.Top(); // Best node according to f(n) in O
        //
        //     if (w == b) {
        //         // Code to find second worst leaf according to c(n) goes here
        //         // Assign the second worst leaf to w
        //     }
        //     else {
        //         w = O.Back();
        //         w.RemoveLast();
        //     }
        //
        //     return w;
        // }

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