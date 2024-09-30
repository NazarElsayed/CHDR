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

        struct ESMASNode final : IHeapItem, std::enable_shared_from_this<ESMASNode> {

            friend std::shared_ptr<ESMASNode>;

            size_t m_Depth;
            size_t m_Coord;

            Ts m_GScore;
            Ts m_FScore;

            std::shared_ptr<ESMASNode> m_Parent;

            std::vector<std::shared_ptr<ESMASNode>> m_Successors;

            std::unordered_map<size_t, Ts> m_ForgottenFCosts;

        private:

            [[nodiscard]] constexpr ESMASNode(const size_t& _depth, const size_t& _coord, const Ts& _gScore, const Ts& _hScore, const std::shared_ptr<ESMASNode>& _parent) : IHeapItem(),
                m_Depth (_depth),
                m_Coord (_coord),
                m_GScore(_gScore),
                m_FScore(_gScore + _hScore),
                m_Parent(_parent),
                m_ForgottenFCosts() {}

        public:

            bool HasActiveNeighbors(const Mazes::Grid<Kd, Tm>& _maze) {

                bool result = false;

                for (auto& neighbour : _maze.GetNeighbours(m_Coord)) {

                    if (const auto& [nActive, nCoord] = neighbour; nActive) {
                        result = true;
                        break;
                    }
                }

                return result;
            }

            void Shrink() {
                m_Successors.clear();
            }

            auto& Expand(const Mazes::Grid<Kd, Tm>& _maze, const coord_t& _end, Ts (*_h)(const coord_t&, const coord_t&)) {

                if (m_Successors.empty()) {

                    const auto neighbours = _maze.GetNeighbours(m_Coord);
                    m_Successors.reserve(neighbours.size());

                    size_t index = 0U;

                    // Expand b and assign its successors to N
                    for (auto& neighbour : neighbours) {

                        if (const auto& [nActive, nCoord] = neighbour; nActive) {

                            const auto n = Utils::To1D(nCoord, _maze.Size());

                            if (m_Parent == nullptr || m_Parent->m_Coord != n) {

                                m_Successors.emplace_back(ESMASNode::CreateShared(
                                    m_Depth  + 1U,              // Depth
                                    n,                          // Coordinate
                                    m_GScore + 1U,              // G-Score
                                    _h(nCoord, _end),           // F-Score
                                    this->shared_from_this()    // Parent
                                ));

                                ++index;
                            }
                        }
                    }

                    m_Successors.shrink_to_fit();
                }

                return m_Successors;
            }

            template<typename... Args>
            static constexpr std::shared_ptr<ESMASNode> CreateShared(Args&&... args) {

                auto result = std::shared_ptr<ESMASNode>(
                    new ESMASNode(std::forward<Args>(args)...),
                    [](ESMASNode *_ptr) {
                        while (_ptr->m_Parent && static_cast<unsigned>(_ptr->m_Parent.use_count()) < 2U) {
                            _ptr->m_Parent = std::move(_ptr->m_Parent->m_Parent);
                        }
                        delete _ptr;
                    }
                );

//                result->m_Parent->m_Successors.emplace_back(result);

                return result;
            }

            [[nodiscard]] constexpr bool operator == (const std::shared_ptr<ESMASNode>& _node) const { return m_Coord == _node->m_Coord; }

            struct Max {

                [[nodiscard]] constexpr bool operator () (const std::shared_ptr<ESMASNode>& _a, const std::shared_ptr<ESMASNode>& _b) const {

                    bool result{};

                    if (_a->m_FScore == _b->m_FScore) {
                        result = _a->m_GScore > _b->m_GScore;
                    }
                    else {
                        result = _a->m_FScore > _b->m_FScore;
                    }

                    return result;
                }
            };
        };

    public:

        void Solve(const Mazes::IMaze<Tm>& _maze) override {

            (void)_maze; // Suppress unused variable warning.

            throw std::runtime_error("ESMAStar::Solve(const Mazes::IMaze& _maze): Not implemented!");
        }

        auto Solve(const Mazes::Grid<Kd, Tm>& _maze, const coord_t& _start, const coord_t& _end, Ts (*_h)(const coord_t&, const coord_t&), const size_t& _memoryLimit) const {

            /** @see: https://easychair.org/publications/paper/TL2M/open */

            std::vector<coord_t> result;

            const auto s = Utils::To1D(_start, _maze.Size());
            const auto e = Utils::To1D(_end,   _maze.Size());

            Heap<std::shared_ptr<ESMASNode>, typename ESMASNode::Max> openSet;
            openSet.Emplace(ESMASNode::CreateShared(
                0U,                 // Depth
                s,                  // Coordinate
                static_cast<Ts>(0), // G-Score
                _h(_start, _end),   // F-Score
                std::shared_ptr<ESMASNode>(nullptr)
            ));

            enum State : char {
                NOMINAL,
                IMPOSSIBLE,
                OUTOFMEMORY
            };

            State state(NOMINAL);

            // Main loop
            while (!openSet.Empty()) {

                auto current(std::move(openSet.Top())); // Node with smallest f-cost in O
                openSet.RemoveFirst();

                if (current->m_Coord != e) { // SEARCH FOR SOLUTION...

                    if (current->m_FScore != std::numeric_limits<Ts>::infinity()) {

                        auto successors_current = current->Expand(_maze, _end, _h);

                        // Get successors of current:

                        for (size_t i = 0U; i < successors_current.size(); ++i) {

                            auto& successor = successors_current[i];

                            auto search = current->m_ForgottenFCosts.find(successor->m_Coord);
                            if (search != current->m_ForgottenFCosts.end()) {   /* condition to check if s(n) is in forgotten f-cost table of b*/

                                const auto& [nCoord, nCost] = *search;

                                successor->m_FScore = nCost;                // f-value of s(n) in forgotten f-cost table of node b
                                current->m_ForgottenFCosts.erase(nCoord);   // Remove s(n) from forgotten f-cost table of node b.
                            }
                            else {

                                /*
                                 * Update the state accordingly:
                                 *         Has no successors:   IMPOSSIBLE
                                 *      Memory Limit Reached:   OUTOFMEMORY
                                 *                 Otherwise:   NOMINAL
                                 */
                                state = (
                                    !successor->HasActiveNeighbors(_maze) ? IMPOSSIBLE : (
                                        successor->m_Depth >= _memoryLimit - 1U ?
                                            OUTOFMEMORY :
                                            NOMINAL
                                    )
                                );

                                if (successor->m_Coord != e && state != NOMINAL) {
                                    successor->m_FScore = std::numeric_limits<Ts>::infinity();

                                    successors_current.erase(successors_current.begin() + i);
                                }
                                else {
                                    successor->m_FScore = std::max(current->m_FScore, successor->m_GScore + _h(Utils::ToND(successor->m_Coord, _maze.Size()), _end));
                                }
                            }

                            // Add n to O.
                            if (successor != nullptr && !openSet.Contains(successor)) {
                                openSet.Add(successor);
                            }
                        }

                        while (openSet.Size() > _memoryLimit) {
                            cull_worst_leaf(_maze, _end, _h, openSet);
                        }

                        current->Shrink();
                    }
                    else {
                        break; // FAILURE.
                    }
                }
                else { // SOLUTION REACHED ...

                    // Free data which is no longer relevant:
                    openSet.Clear();

                    if (state == NOMINAL) {

                        // Recurse from end node to start node, inserting into a result buffer:
                        result.reserve(current->m_GScore);
                        result.emplace_back(Utils::ToND(current->m_Coord, _maze.Size()));

                        if (auto item = current->m_Parent) {

                            while (const auto item_parent = item->m_Parent) {
                                result.emplace_back(Utils::ToND(item->m_Coord, _maze.Size()));

                                auto oldItem = item;
                                item = item_parent;
                                oldItem.reset();
                            }
                        }

                        // Reverse the result:
                        std::reverse(result.begin(), result.end());
                    }

                    break;
                }
            }

            return result;
        }

        void cull_worst_leaf(const Mazes::Grid<Kd, Tm>& _maze, const coord_t& _end, Ts (*_h)(const coord_t&, const coord_t&), Heap<std::shared_ptr<ESMASNode>, typename ESMASNode::Max>& _openSet) const {

            const auto w = safe_culling_heuristic(_openSet);

            if (auto p = w->m_Parent) { // parent node of w

                // Remove w from the successor list of p
                auto p_successors = p->Expand(_maze, _end, _h);

                for (size_t i = 0U; i < p_successors.size(); ++i) {

                    // Code to remove w from the successor list of p goes here
                    if (p_successors[i]->m_Coord == w->m_Coord) {
                        p_successors.erase(p_successors.begin() + i);
                        break;
                    }
                }

                // Add s(w) to forgotten f-cost table of p, with value of f (w)
                p->m_ForgottenFCosts.insert_or_assign(p->m_Coord, w->m_FScore);

                // f (p) ← min of forgotten f-costs of p
                for (const auto& [pState, pCost] : p->m_ForgottenFCosts) {
                    p->m_FScore = std::min(p->m_FScore, pCost);
                }

                // if p is not in _openSet then
                if (!_openSet.Contains(p)) {
                    _openSet.Add(p); // Add p to _openSet
                }
            }
        }

        auto safe_culling_heuristic(Heap<std::shared_ptr<ESMASNode>, typename ESMASNode::Max>& _openSet) const {

            auto w = _openSet.Back(); // Worst leaf according to c(n) in _openSet

            if (w == _openSet.Top()) { // Top == Best node according to f(n) in _openSet

                // Code to find second worst leaf according to c(n) goes here

                w = _openSet.Back();

                for (size_t i = _openSet.Size() / 2U; i < _openSet.Size(); ++i) {

                    const auto& A = _openSet[i];
                    const auto& B = w;

                    if (typename ESMASNode::Max()(A, B)) {
                        w = _openSet[i]; // Assign the second worst leaf to w
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