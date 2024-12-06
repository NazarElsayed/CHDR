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

#include "base/bsolver.hpp"
#include "mazes/grid.hpp"
#include "mazes/base/imaze.hpp"
#include "types/existence_set.hpp"
#include "types/heap.hpp"
#include "utils/utils.hpp"

namespace chdr::solvers {

    template<typename weight_t, const size_t Kd, typename scalar_t, typename index_t>
    class [[maybe_unused]] esmgstar final : public bsolver<weight_t, Kd, scalar_t, index_t> {

        static_assert(std::is_integral_v<scalar_t> || std::is_floating_point_v<scalar_t>, "scalar_t must be either an integral or floating point type");
        static_assert(std::is_integral_v<index_t>, "index_t must be an integral type.");

    private:

        using coord_t = coord<index_t, Kd>;

        struct esmg_node final : std::enable_shared_from_this<esmg_node> {

            friend std::shared_ptr<esmg_node>;

            size_t m_depth;
            index_t m_index;

            scalar_t m_gScore;
            scalar_t m_fScore;

            std::shared_ptr<esmg_node> m_parent;

            std::vector<std::shared_ptr<esmg_node>> m_successors;

            std::unordered_map<size_t, scalar_t> m_forgottenFCosts;

        private:

            [[nodiscard]] constexpr esmg_node(const size_t& _depth, const index_t& _index, const scalar_t& _gScore, const scalar_t& _hScore) :
                m_depth (_depth),
                m_index (_index),
                m_gScore(_gScore),
                m_fScore(_gScore + _hScore),
                m_parent(),
                m_forgottenFCosts() {}

            [[nodiscard]] constexpr esmg_node(const size_t& _depth, const index_t& _index, const scalar_t& _gScore, const scalar_t& _hScore, const std::shared_ptr<esmg_node>&& _parent) :
                m_depth (_depth),
                m_index (_index),
                m_gScore(_gScore),
                m_fScore(_gScore + _hScore),
                m_parent(std::move(_parent)),
                m_forgottenFCosts() {}

        public:

            /**
             * @brief Constructs an uninitialized ESMGSNode.
             *
             * This constructor creates an ESMGSNode with uninitialized members.
             */
            [[nodiscard]] constexpr esmg_node() {} // NOLINT(*-pro-type-member-init, *-use-equals-default)

            void shrink() {

                if (!m_successors.empty()) {

                    for (auto successor : m_successors) {
                        m_forgottenFCosts.insert_or_assign(successor->m_index, successor->m_fScore);
                    }

                    m_successors.clear();
                    m_successors.shrink_to_fit();
                }
            }

            auto& expand(const mazes::grid<Kd, weight_t>& _maze, const coord_t& _end, scalar_t (*_h)(const coord_t&, const coord_t&), const scalar_t& _weight, const size_t& _memoryLimit) {

                if (m_successors.empty()) {

                    const auto neighbours = _maze.get_neighbours(m_index);
                    m_successors.reserve(neighbours.size());

                    for (auto& neighbour : neighbours) {

                        if (const auto& [nActive, nCoord] = neighbour; nActive) {

                            const auto n = utils::to_1d(nCoord, _maze.size());

                            if ((m_parent == nullptr || m_parent->m_index != n) && m_depth + 1U < _memoryLimit) {

                                // Check for any potential successor for the child before its creation.
                                for (auto& successor_neighbours : _maze.get_neighbours(n)) {

                                    if (const auto& [snActive, snCoord] = successor_neighbours; snActive) {

                                        m_successors.emplace_back(esmg_node::create_shared(
                                                m_depth + 1U,              // Depth
                                                n,                          // Coordinate
                                                m_gScore + 1U,              // G-Score
                                                _h(nCoord, _end) * _weight, // F-Score
                                                this->shared_from_this()    // Parent
                                        ));

                                        break;
                                    }
                                }
                            }
                        }
                    }

                    m_successors.shrink_to_fit();
                }

                return m_successors;
            }

            template<typename... Args>
            static constexpr std::shared_ptr<esmg_node> create_shared(Args&&... _args) {

                auto result = std::shared_ptr<esmg_node>(
                    new esmg_node(std::forward<Args>(_args)...),
                    [](esmg_node* _ptr) {

                        while (_ptr->m_parent && static_cast<unsigned>(_ptr->m_parent.use_count()) < 2U) {

                            if (_ptr->m_parent) {

                                for (size_t i = 0U; i < _ptr->m_parent->m_successors.size(); ++i) {

                                    _ptr->m_parent->m_forgottenFCosts.erase(_ptr->m_index);

                                    if (_ptr->m_parent->m_successors[i].get() == _ptr) {
                                        _ptr->m_parent->m_successors.erase(_ptr->m_parent->m_successors.begin() + i);

                                        break;
                                    }
                                }
                            }

                            _ptr->m_parent = std::move(_ptr->m_parent->m_parent);
                        }

                        delete _ptr;
                    }
                );

                if (result->m_parent) {
                    result->m_parent->m_successors.emplace_back(result);
                }

                return result;
            }

            [[nodiscard]] constexpr bool operator == (const std::shared_ptr<esmg_node>& _node) const { return m_index == _node->m_index; }

            struct max {

                [[nodiscard]] constexpr bool operator () (const std::shared_ptr<esmg_node>& _a, const std::shared_ptr<esmg_node>& _b) const {
                    return _a->m_fScore == _b->m_fScore ?
                        _a->m_gScore > _b->m_gScore :
                        _a->m_fScore > _b->m_fScore;
                }
            };

            struct min {

                [[nodiscard]] constexpr bool operator () (const std::shared_ptr<esmg_node>& _a, const std::shared_ptr<esmg_node>& _b) const {
                    return _a->m_fScore == _b->m_fScore ?
                        _a->m_gScore < _b->m_gScore :
                        _a->m_fScore < _b->m_fScore;
                }
            };
        };

        using heap_t = heap<std::shared_ptr<esmg_node>, typename esmg_node::min>;

        void cull_worst_leaf(const mazes::grid<Kd, weight_t>& _maze, const coord_t& _end, scalar_t (*_h)(const coord_t&, const coord_t&), const scalar_t& _weight, const size_t& _memoryLimit, heap_t& _open) const {

            const auto w = safe_culling_heuristic(_open);

            if (auto p = w->m_parent) { // parent node of w

                // Remove w from the successor list of p
                auto p_successors = p->expand(_maze, _end, _h, _weight, _memoryLimit);

                for (size_t i = 0U; i < p_successors.size(); ++i) {

                    // Code to remove w from the successor list of p goes here
                    if (p_successors[i]->m_index == w->m_index) {
                        p_successors.erase(p_successors.begin() + i);
                        break;
                    }
                }

                // Add s(w) to forgotten f-cost table of p, with value of f (w)
                p->m_forgottenFCosts.insert_or_assign(w->m_index, w->m_fScore);

                // f (p) ← min of forgotten f-costs of p
                for (const auto& [pState, pCost] : p->m_forgottenFCosts) {
                    p->m_fScore = std::min(p->m_fScore, pCost);
                }

                // if p is not in _open then
                if (!_open.contains(p)) {
                     _open.add(p); // Add p to _open
                }
            }
        }

        [[nodiscard]] auto safe_culling_heuristic(heap_t& _open) const {

            auto w = _open.back(); // Worst leaf according to c(n) in _open

            if (w == _open.top()) { // Top == Best node according to f(n) in _open

                // Code to find second-worst leaf according to c(n) goes here

                w = _open.back();

                for (size_t i = _open.size() / 2U; i < _open.size(); ++i) {

                    const auto& a = _open[i];
                    const auto& b = w;

                    if (typename esmg_node::max()(a, b)) {
                        w = _open[i]; // Assign the second-worst leaf to w
                    }
                }

                _open.remove(w);
            }
            else {
                _open.pop_back();
            }

            return w;
        }

    public:

        [[maybe_unused]]
        auto solve(const mazes::grid<Kd, weight_t>& _maze, const coord_t& _start, const coord_t& _end, scalar_t (*_h)(const coord_t&, const coord_t&), const scalar_t& _weight = 1, const size_t& _memoryLimit = -1U) const {

            /** @see: https://easychair.org/publications/paper/TL2M/open */

            std::vector<coord_t> result;

            const auto s = utils::to_1d(_start, _maze.size());
            const auto e = utils::to_1d(_end, _maze.size());

            // Create Open Set:
            heap_t open;
            open.emplace(esmg_node::create_shared(
                0U,                         // Depth
                s,                          // Coordinate
                static_cast<scalar_t>(0),   // G-Score
                _h(_start, _end) * _weight  // F-Score
            ));

            // Main loop:
            while (!open.empty()) {

                auto curr(std::move(open.top()));
                open.pop(); // Node with smallest f-cost in O

                debug::log(std::to_string(curr->m_index ));
                debug::log(std::to_string(curr->m_fScore));

                if (curr->m_index != e) { // SEARCH FOR SOLUTION...

                    auto successors_current = curr->expand(_maze, _end, _h, _weight, _memoryLimit);

                    for (size_t i = 0U; i < successors_current.size(); ++i) {

                        auto& successor = successors_current[i];

                        // Check if s(n) is in forgotten f-cost table of b.
                        auto search = curr->m_forgottenFCosts.find(successor->m_index);
                        if (search != curr->m_forgottenFCosts.end()) {

                            const auto& [nCoord, nCost] = *search;

                            successor->m_fScore = nCost;
                            curr->m_forgottenFCosts.erase(nCoord);
                        }
                        else {
                            successor->m_fScore = std::max(curr->m_fScore, successor->m_gScore + (_h(utils::to_nd(successor->m_index, _maze.size()), _end) * _weight));
                        }

                        // Add successor to open.
                        if (!open.contains(successor)) {
                             open.add(successor);
                        }
                    }

                    while (open.size() > _memoryLimit) {
                        cull_worst_leaf(_maze, _end, _h, _weight, _memoryLimit, open);
                    }

                    // Shrink the node to release ownership of children, allowing automatic GC of parents with no valid candidate children.
                    curr->shrink();
                }
                else { // SOLUTION REACHED ...

                    // Free data which is no longer relevant:
                    open.clear(); open.shrink_to_fit();

                    if (curr != nullptr) {

                        // reserve space in result:
                        result.reserve(curr->m_gScore);

                        // Recurse from end node to start node, inserting into a result buffer:
                        result.emplace_back(utils::to_nd(curr->m_index, _maze.size()));

                        if (auto item = curr->m_parent) {

                            while (const auto item_parent = item->m_parent) {
                                result.emplace_back(utils::to_nd(item->m_index, _maze.size()));

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

    };

} // chdr::solvers

#endif //CHDR_ESMGSTAR_HPP