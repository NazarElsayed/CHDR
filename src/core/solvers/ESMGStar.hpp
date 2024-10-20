/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_ESMGSTAR_HPP
#define CHDR_ESMGSTAR_HPP

#include <cmath>
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
    class ESMGStar final {

        static_assert(std::is_integral_v<Ts> || std::is_floating_point_v<Ts>, "Ts must be either an integral or floating point type");

    private:

        using coord_t = Coord<size_t, Kd>;

        struct ESMASNode final : IHeapItem, std::enable_shared_from_this<ESMASNode> {

            friend std::shared_ptr<ESMASNode>;

            size_t m_Depth;
            size_t m_Index;

            Ts m_GScore;
            Ts m_FScore;

            std::shared_ptr<ESMASNode> m_Parent;

            std::vector<std::shared_ptr<ESMASNode>> m_Successors;

            std::unordered_map<size_t, Ts> m_ForgottenFCosts;

        private:

            [[nodiscard]] constexpr ESMASNode(const size_t& _depth, const size_t& _coord, const Ts& _gScore, const Ts& _hScore, const std::shared_ptr<ESMASNode>& _parent) : IHeapItem(),
                m_Depth (_depth),
                m_Index (_coord),
                m_GScore(_gScore),
                m_FScore(_gScore + _hScore),
                m_Parent(_parent),
                m_ForgottenFCosts() {}

        public:

            void Shrink() {

                if (!m_Successors.empty()) {

                    for (auto successor : m_Successors) {
                        m_ForgottenFCosts.insert_or_assign(successor->m_Index, successor->m_FScore);
                    }

                    m_Successors.clear();
                    m_Successors.shrink_to_fit();
                }
            }

            auto& Expand(const Mazes::Grid<Kd, Tm>& _maze, const coord_t& _end, Ts (*_h)(const coord_t&, const coord_t&), const Ts& _weight, const size_t& _memoryLimit) {

                if (m_Successors.empty()) {

                    const auto neighbours = _maze.GetNeighbours(m_Index);
                    m_Successors.reserve(neighbours.size());

                    for (auto& neighbour : neighbours) {

                        if (const auto& [nActive, nCoord] = neighbour; nActive) {

                            const auto n = Utils::To1D(nCoord, _maze.Size());

                            if ((m_Parent == nullptr || m_Parent->m_Index != n) && m_Depth + 1U < _memoryLimit) {

                                // Check for any potential successor for the child before its creation.
                                for (auto& successor_neighbours : _maze.GetNeighbours(n)) {

                                    if (const auto& [snActive, snCoord] = successor_neighbours; snActive) {

                                        m_Successors.emplace_back(ESMASNode::CreateShared(
                                            m_Depth  + 1U,              // Depth
                                            n,                          // Coordinate
                                            m_GScore + 1U,              // G-Score
                                            _h(nCoord, _end) * _weight, // F-Score
                                            this->shared_from_this()    // Parent
                                        ));

                                        break;
                                    }
                                }
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

                            if (_ptr->m_Parent) {

                                for (size_t i = 0U; i < _ptr->m_Parent->m_Successors.size(); ++i) {

                                    _ptr->m_Parent->m_ForgottenFCosts.erase(_ptr->m_Index);

                                    if (_ptr->m_Parent->m_Successors[i].get() == _ptr) {
                                        _ptr->m_Parent->m_Successors.erase(_ptr->m_Parent->m_Successors.begin() + i);

                                        break;
                                    }
                                }
                            }

                            _ptr->m_Parent = std::move(_ptr->m_Parent->m_Parent);
                        }

                        delete _ptr;
                    }
                );

                if (result->m_Parent) {
                    result->m_Parent->m_Successors.emplace_back(result);
                }

                return result;
            }

            [[nodiscard]] constexpr bool operator == (const std::shared_ptr<ESMASNode>& _node) const { return m_Index == _node->m_Index; }

            struct Max {

                [[nodiscard]] constexpr bool operator () (const std::shared_ptr<ESMASNode>& _a, const std::shared_ptr<ESMASNode>& _b) const {
                    return _a->m_FScore == _b->m_FScore ?
                        _a->m_GScore > _b->m_GScore :
                        _a->m_FScore > _b->m_FScore;
                }
            };
        };

    public:

        auto Solve(const Mazes::Grid<Kd, Tm>& _maze, const coord_t& _start, const coord_t& _end, Ts (*_h)(const coord_t&, const coord_t&), const Ts& _weight = 1, const size_t& _memoryLimit = -1U) const {

            /** @see: https://easychair.org/publications/paper/TL2M/open */

            std::vector<coord_t> result;

            const auto s = Utils::To1D(_start, _maze.Size());
            const auto e = Utils::To1D(_end,   _maze.Size());

            if (_maze.Contains(s) &&
                _maze.Contains(e) &&
                _maze.At(s).IsActive() &&
                _maze.At(e).IsActive()
            ) {

                if (s != e) {

                    Heap<std::shared_ptr<ESMASNode>, typename ESMASNode::Max> openSet;
                    openSet.Emplace(ESMASNode::CreateShared(
                        0U,                         // Depth
                        s,                          // Coordinate
                        static_cast<Ts>(0),         // G-Score
                        _h(_start, _end) * _weight, // F-Score
                        nullptr
                    ));

                    // Main loop
                    while (!openSet.Empty()) {

                        auto current(std::move(openSet.Top())); // Node with smallest f-cost in O
                        openSet.RemoveFirst();

                        if (current->m_Index != e) { // SEARCH FOR SOLUTION...

                            auto successors_current = current->Expand(_maze, _end, _h, _weight, _memoryLimit);

                            for (size_t i = 0U; i < successors_current.size(); ++i) {

                                auto& successor = successors_current[i];

                                // Check if s(n) is in forgotten f-cost table of b.
                                auto search = current->m_ForgottenFCosts.find(successor->m_Index);
                                if (search != current->m_ForgottenFCosts.end()) {

                                    const auto& [nCoord, nCost] = *search;

                                    successor->m_FScore = nCost;
                                    current->m_ForgottenFCosts.erase(nCoord);
                                }
                                else {
                                    successor->m_FScore = std::max(current->m_FScore, successor->m_GScore + _h(Utils::ToND(successor->m_Index, _maze.Size()), _end) * _weight);
                                }

                                // Add successor to openSet.
                                if (!openSet.Contains(successor)) {
                                     openSet.Add(successor);
                                }
                            }

                            while (openSet.Size() > _memoryLimit) {
                                cull_worst_leaf(_maze, _end, _h, _weight, _memoryLimit, openSet);
                            }

                            // Shrink the node to release ownership of children, allowing automatic GC of parents with no valid candidate children.
                            current->Shrink();
                        }
                        else { // SOLUTION REACHED ...

                            // Free data which is no longer relevant:
                            openSet.Clear();

                            if (current != nullptr) {

                                // Recurse from end node to start node, inserting into a result buffer:
                                result.reserve(current->m_GScore);
                                result.emplace_back(Utils::ToND(current->m_Index, _maze.Size()));

                                if (auto item = current->m_Parent) {

                                    while (const auto item_parent = item->m_Parent) {
                                        result.emplace_back(Utils::ToND(item->m_Index, _maze.Size()));

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
                }
                else {
                    result.emplace_back(_end);
                }
            }

            return result;
        }

        void cull_worst_leaf(const Mazes::Grid<Kd, Tm>& _maze, const coord_t& _end, Ts (*_h)(const coord_t&, const coord_t&), const Ts& _weight, const size_t& _memoryLimit, Heap<std::shared_ptr<ESMASNode>, typename ESMASNode::Max>& _openSet) const {

            const auto w = safe_culling_heuristic(_openSet);

            if (auto p = w->m_Parent) { // parent node of w

                // Remove w from the successor list of p
                auto p_successors = p->Expand(_maze, _end, _h, _weight, _memoryLimit);

                for (size_t i = 0U; i < p_successors.size(); ++i) {

                    // Code to remove w from the successor list of p goes here
                    if (p_successors[i]->m_Index == w->m_Index) {
                        p_successors.erase(p_successors.begin() + i);
                        break;
                    }
                }

                // Add s(w) to forgotten f-cost table of p, with value of f (w)
                p->m_ForgottenFCosts.insert_or_assign(w->m_Index, w->m_FScore);

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

#endif //CHDR_ESMGSTAR_HPP