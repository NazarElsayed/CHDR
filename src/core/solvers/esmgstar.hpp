/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_ESMGSTAR_HPP
#define CHDR_ESMGSTAR_HPP

#include <debug.hpp>

#include <cstddef>

#include "base/managed_node.hpp"
#include "mazes/grid.hpp"
#include "utils/utils.hpp"

namespace chdr::solvers {

    template<size_t Kd, typename scalar_t, typename index_t, typename params_t>
    struct [[maybe_unused]] esmgstar final {

        friend struct solver<esmgstar, Kd, scalar_t, index_t, params_t>;

        static_assert(std::is_integral_v<scalar_t> || std::is_floating_point_v<scalar_t>, "scalar_t must be either an integral or floating point type");
        static_assert(std::is_integral_v<index_t>, "index_t must be an integral type.");

    private:

        using solver_t = solver<esmgstar, Kd, scalar_t, index_t, params_t>;
        using  coord_t = coord<index_t, Kd>;
        using weight_t = typename params_t::weight_type;

        struct node final : std::enable_shared_from_this<node> {

            friend std::shared_ptr<node>;

            std::shared_ptr<node> m_parent;

            size_t m_depth;
            index_t m_index;

            scalar_t m_gScore;
            scalar_t m_fScore;

            std::vector<std::shared_ptr<node>> m_successors;

            std::unordered_map<size_t, scalar_t> m_forgottenFCosts;

            [[nodiscard]] constexpr node() : managed_node<index_t>() {}

            [[nodiscard]] constexpr node(const size_t& _depth, const index_t& _index, const scalar_t& _gScore, const scalar_t& _hScore) :
                m_depth (_depth),
                m_index (_index),
                m_gScore(_gScore),
                m_fScore(_gScore + _hScore),
                m_parent() {}

            [[nodiscard]] constexpr node(const size_t& _depth, const index_t& _index, const scalar_t& _gScore, const scalar_t& _hScore, const std::shared_ptr<node>& _parent) :
                m_depth (_depth),
                m_index (_index),
                m_gScore(_gScore),
                m_fScore(_gScore + _hScore),
                m_parent(_parent) {}

            template<typename node_t, typename coord_t>
            [[nodiscard]] auto backtrack(const coord_t& _size, const size_t& _capacity = 0U) {

                const auto& curr = *static_cast<const node_t*>(this);

                // Recurse from end node to start node, inserting into a result buffer:
                std::vector<coord_t> result;
                result.reserve(_capacity);
                result.emplace_back(utils::to_nd(curr.m_index, _size));

                if (curr.m_parent != nullptr) {

                    for (auto t = curr.m_parent; t->m_parent != nullptr;) {
                        result.emplace_back(utils::to_nd(t->m_index, _size));

                        auto oldItem = t;
                        t = t->m_parent;
                        oldItem.reset();
                    }
                }

                // Reverse the result:
                std::reverse(result.begin(), result.end());

                return result;
            }

            constexpr void shrink() {

                if (!m_successors.empty()) {

                    for (auto successor : m_successors) {
                        m_forgottenFCosts.insert_or_assign(successor->m_index, successor->m_fScore);
                    }

                    m_successors.clear();
                    m_successors.shrink_to_fit();
                }
            }

            constexpr auto& expand(const mazes::grid<Kd, weight_t>& _maze, const coord_t& _end, scalar_t (*_h)(const coord_t&, const coord_t&), const scalar_t& _weight, const size_t& _memoryLimit) {

                if (m_successors.empty()) {

                    const auto neighbours = _maze.get_neighbours(m_index);
                    m_successors.reserve(neighbours.size());

                    for (auto& neighbour : neighbours) {

                        if (const auto& [nActive, nCoord] = neighbour; nActive) {

                            const auto n = utils::to_1d(nCoord, _maze.size());

                            if ((this->m_parent == nullptr || this->m_parent->m_index != n) && m_depth + 1U < _memoryLimit) {

                                // Check for any potential successor for the child before its creation.
                                for (auto& successor_neighbours : _maze.get_neighbours(n)) {

                                    if (const auto& [snActive, snCoord] = successor_neighbours; snActive) {

                                        m_successors.emplace_back(node::create_shared(
                                                m_depth + 1U,               // Depth
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

            template <typename... Args>
            static constexpr std::shared_ptr<node> create_shared(Args&&... _args) {

                auto result = std::shared_ptr<node>(
                    new node(std::forward<Args>(_args)...),
                    [](node* _ptr) {

                        while (_ptr->m_parent && static_cast<unsigned>(_ptr->m_parent.use_count()) < 2U) {

                            if (auto node_parent = std::dynamic_pointer_cast<node>(_ptr->m_parent)) {

                                for (size_t i = 0U; i < node_parent->m_successors.size(); ++i) {
                                    node_parent->m_forgottenFCosts.erase(_ptr->m_index);

                                    if (node_parent->m_successors[i].get() == _ptr) {
                                        node_parent->m_successors.erase(node_parent->m_successors.begin() + i);

                                        break;
                                    }
                                }

                                _ptr->m_parent = std::move(node_parent->m_parent);
                            }
                            else {
                                break;
                            }
                        }

                        delete _ptr;
                    }
                );

                if (auto node_parent = std::dynamic_pointer_cast<node>(result->m_parent)) {
                    node_parent->m_successors.emplace_back(result);
                }

                return result;
            }

            [[nodiscard]] friend constexpr bool operator < (const node& _a, const node& _b) noexcept {
                return _a->m_fScore == _b->m_fScore ?
                       _a->m_gScore >  _b->m_gScore :
                       _a->m_fScore >  _b->m_fScore;
            }
        };

        using open_set_t = heap<std::shared_ptr<node>>;

        static constexpr void cull_worst_leaf(const mazes::grid<Kd, weight_t>& _maze, const coord_t& _end, scalar_t (*_h)(const coord_t&, const coord_t&), const scalar_t& _weight, const size_t& _memoryLimit, open_set_t& _open) {

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
                     _open.push(p); // Add p to _open
                }
            }
        }

        [[nodiscard]] static constexpr auto safe_culling_heuristic(open_set_t& _open) {

            auto w = _open.back(); // Worst leaf according to c(n) in _open

            if (w == _open.top()) { // Top == Best node according to f(n) in _open

                // Code to find second-worst leaf according to c(n) goes here

                w = _open.back();

                for (size_t i = _open.size() / 2U; i < _open.size(); ++i) {

                    const auto& a = _open[i];
                    const auto& b = w;

                    if (a < b) {
                        w = _open[i]; // Assign the second-worst leaf to w
                    }
                }

                _open.erase(w);
            }
            else {
                _open.pop_back();
            }

            return w;
        }

        [[nodiscard]] static constexpr auto solve_internal(open_set_t& _open, const params_t& _params) {

            /** @see: https://easychair.org/publications/paper/TL2M/open */

            const auto s = utils::to_1d(_params.start, _params.size);
            const auto e = utils::to_1d(_params.end,   _params.size);

            // Create Open Set:
            _open.emplace(node::create_shared(
                0U,                                                     // Depth
                s,                                                      // Coordinate
                static_cast<scalar_t>(0),                               // G-Score
                _params.h(_params.start, _params.end) * _params.weight  // F-Score
            ));

            // Main loop:
            while (!_open.empty()) {

                auto curr(std::move(_open.top()));
                _open.pop();

                if (curr->m_index != e) { // SEARCH FOR SOLUTION...

                    auto successors_current = curr->expand(_params.maze, _params.end, _params.h, _params.weight, _params.memoryLimit);

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
                            successor->m_fScore = std::max(curr->m_fScore, successor->m_gScore + (_params.h(utils::to_nd(successor->m_index, _params.size), _params.end) * _params.weight));
                        }

                        // Add successor to open.
                        if (!_open.contains(successor)) {
                             _open.emplace(successor);
                        }
                    }

                    while (_open.size() > _params.memoryLimit) {
                        cull_worst_leaf(_params.maze, _params.end, _params.h, _params.weight, _params.memoryLimit, _open);
                    }

                    // Shrink the node to release ownership of children, allowing automatic GC of parents with no valid candidate children.
                    curr->shrink();
                }
                else { // SOLUTION REACHED ...

                    // Free data which is no longer relevant:
                    _open.clear();
                    _open.shrink_to_fit();

                    return curr->template backtrack<node>(_params.size, curr->m_gScore);
                }
            }

            return std::vector<coord_t>{};
        }

        [[maybe_unused, nodiscard]] static constexpr auto execute(const params_t& _params) {

            open_set_t open;

            return solve_internal(open, _params);
        }
    };

} //chdr::solvers

#endif //CHDR_ESMGSTAR_HPP