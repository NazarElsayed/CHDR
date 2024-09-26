/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_ESMASTAR_HPP
#define CHDR_ESMASTAR_HPP

#include <cmath>
#include <functional>
#include <queue>
#include <unordered_set>

#include "../utils/Heuristics.hpp"
#include "base/ISolver.hpp"
#include "mazes/base/IMaze.hpp"
#include "mazes/Grid.hpp"
#include "types/ExistenceSet.hpp"
#include "types/Heap.hpp"
#include "utils/Utils.hpp"

namespace CHDR::Solvers {

    template<typename Tm, const size_t Kd, typename Ts>
    class ESMAStar final : public ISolver<Tm> {

        static_assert(std::is_integral_v<Ts> || std::is_floating_point_v<Ts>, "Ts must be either an integral or floating point type");

    private:

        using coord_t = Coord<size_t, Kd>;

        struct ESMASNode final : IHeapItem {

            bool m_Expanded;

            size_t m_Depth;
            size_t m_Coord;

            Ts m_GScore;
            Ts m_FScore;

            ESMASNode* m_Parent;

            std::vector<ESMASNode> m_Successors;

            std::unordered_map<size_t, Ts> m_ForgottenFCosts;

            [[nodiscard]] constexpr ESMASNode(const size_t& _depth, const size_t& _coord, const Ts& _gScore, const Ts& _hScore, ESMASNode* const _parent) : IHeapItem(),
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

            [[nodiscard]] constexpr bool operator == (const ESMASNode& _node) const { return m_Coord == _node.m_Coord; }

            struct Max {

                [[nodiscard]] constexpr bool operator () (const ESMASNode& _a, const ESMASNode& _b) const {

                    bool result{};

                    if (_a.m_FScore == _b.m_FScore) {
                        result = _a.m_GScore > _b.m_GScore;
                    }
                    else {
                        result = _a.m_FScore > _b.m_FScore;
                    }

                    return result;
                }
            };
        };

    public:

        void Solve(const Mazes::IMaze<Tm>& _maze) override {

            (void)_maze; // Suppress unused variable warning.

            throw std::runtime_error("AStar::Solve(const Mazes::IMaze& _maze) Not implemented!");
        }

        auto Solve(const Mazes::Grid<Kd, Tm>& _maze, const coord_t& _start, const coord_t& _end, Ts (*_h)(const coord_t&, const coord_t&), const size_t& _memoryLimit) const {

            (void)_maze; // Suppress unused variable warnings.
            (void)_start;
            (void)_end;
            (void)_h;
            (void)_memoryLimit;

            throw std::runtime_error("Djikstra::Solve(const Mazes::IMaze& _maze) Not implemented!");

//            /** @see: https://easychair.org/publications/open/TL2M */
//
//            std::vector<coord_t> result;
//
//            const auto s = Utils::To1D(_start, _maze.Size());
//            const auto e = Utils::To1D(_end,   _maze.Size());
//
//            Heap<ESMASNode, typename ESMASNode::Max> openSet;
//            openSet.Emplace({
//                0U,                 // Depth
//                s,                  // Coordinate
//                static_cast<Ts>(0), // G-Score
//                _h(_start, _end),   // F-Score
//                nullptr             // Parent
//            });
//
//            size_t u = 1U; // counter for nodes in memory
//
//            // Main loop
//            while (!openSet.Empty()) {
//
//                ESMASNode b = openSet.Top(); // Node with smallest f-cost in O
//                openSet.RemoveFirst();
//
//                // If node b is goal
//                if (b.m_Coord != e) {
//
//                    if (b.m_FScore != std::numeric_limits<Ts>::infinity()) {
//
//                        if (!b.m_Expanded) {
//                            b.m_Expanded = true;
//
//                            // Expand b and assign its successors to N
//                            auto neighbours = _maze.GetNeighbours(b.m_Coord);
//
//                            for (auto& neighbour : neighbours) {
//
//                                if (const auto [nActive, nValue] = neighbour; nActive) {
//
//                                    auto nCoord = Utils::To1D(nValue, _maze.Size());
//
//                                    if (nCoord != b.m_Coord) {
//
//                                        ESMASNode n(
//                                            b.m_Depth + 1U,     // Depth
//                                            nCoord,             // Coordinate
//                                            b.m_GScore + 1U,    // G-Score
//                                            _h(nValue, _end),   // F-Score
//                                            &b                  // Parent
//                                        );
//
//                                        b.m_Successors.emplace_back(std::move(n));
//                                    }
//                                }
//                            }
//
//                        }
//
//                        std::vector<ESMASNode> N = b.m_Successors; // Set N as successors of b
//
//                        for (ESMASNode& n : N) {
//
//                            auto search = b.m_ForgottenFCosts.find(n.m_Coord);
//                            if (search != b.m_ForgottenFCosts.end()) {  /* condition to check if s(n) is in forgotten f-cost table of b*/
//
//                                const auto [nCoord, nCost] = *search;
//
//                                n.m_FScore = nCost;                          // f-value of s(n) in forgotten f-cost table of node b
//                                b.m_ForgottenFCosts.erase(nCoord);           // Remove s(n) from forgotten f-cost table of node b.
//                            }
//                            else if (n.m_Coord != e && (n.m_Successors.empty() || n.m_Depth >= _memoryLimit - 1U)) {
//                                n.m_FScore = std::numeric_limits<Ts>::infinity();
//                            }
//                            else {
//                                // Update properties of n according to the pseudocode
//                                n.m_FScore = std::max(b.m_FScore, n.m_GScore + _h(Utils::ToND(n.m_Coord, _maze.Size()), _end));
//                            }
//
//                            // Add n to O
//                            openSet.Add(n);
//                            u++;
//                        }
//
//                        while (u > _memoryLimit) {
//                            cull_worst_leaf(openSet, u);
//                        }
//                    }
//                    else {
//                        break; // Return goal not found
//                    }
//                }
//                else {
//
//                    /* SOLUTION REACHED */
//
//                    // Free data which is no longer relevant:
//                    openSet.Clear();
//
//                    // Recurse from end node to start node, inserting into a result buffer:
//                    result.reserve(b.m_GScore);
//
//                    for (const auto* temp = &b; temp->m_Parent != nullptr; temp = temp->m_Parent) {
//                        result.emplace_back(Utils::ToND(temp->m_Coord, _maze.Size()));
//                    }
//
//                    // Reverse the result:
//                    std::reverse(result.begin(), result.end());
//
//                    break;
//                }
//            }
//
//            return result;
        }

        void cull_worst_leaf(Heap<ESMASNode, typename ESMASNode::Max>& _openSet, size_t& _memoryLimit) const {

            const ESMASNode w = safe_culling_heuristic(_openSet);

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
            p->m_ForgottenFCosts.insert_or_assign(p->m_Coord, w.m_FScore);

            // f (p) ← min of forgotten f-costs of p
            for (auto& [state, cost] : p->m_ForgottenFCosts) {
                p->m_FScore = std::min(p->m_FScore, cost);
            }

            // if p is not in _openSet then
            if (!_openSet.Contains(*p)) {
                _openSet.Add(*p); // Add p to _openSet
            }

            _memoryLimit--;
        }

        auto safe_culling_heuristic(Heap<ESMASNode, typename ESMASNode::Max>& _openSet) const {

            ESMASNode w = _openSet.Back(); // Worst leaf according to c(n) in _openSet

            const ESMASNode b = _openSet.Top(); // Best node according to f(n) in _openSet

            if (w == b) {

                // Code to find second worst leaf according to c(n) goes here
                // Assign the second worst leaf to w

                w = _openSet.Back();

                for (size_t i = _openSet.Size() / 2U; i < _openSet.Size(); ++i) {

                    const auto& A = _openSet[i];
                    const auto& B = w;

                    if (typename ESMASNode::Max()(A, B)) {
                        w = _openSet[i];
                    }
                }

                _openSet.Remove(w);
            }
            else {
                _openSet.RemoveLast();
            }

            return w;
        }

    };

} // CHDR::Solvers

#endif //CHDR_ESMASTAR_HPP