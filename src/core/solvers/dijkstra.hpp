/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_DIJKSTRA_HPP
#define CHDR_DIJKSTRA_HPP

#include <memory>

#include "mazes/grid.hpp"
#include "mazes/base/imaze.hpp"
#include "types/existence_set.hpp"

namespace chdr::solvers {

    template<size_t Kd, typename scalar_t, typename index_t, typename params_t>
    struct [[maybe_unused]] dijkstra final {

        friend struct solver<dijkstra, Kd, scalar_t, index_t, params_t>;

        static_assert(std::is_integral_v<index_t>, "index_t must be an integral type.");

    private:

        using solver_t = solver<dijkstra, Kd, scalar_t, index_t, params_t>;
        using  coord_t = coord<index_t, Kd>;

        struct node final {

            index_t m_index;

            scalar_t m_gScore;
            scalar_t m_fScore;

            std::shared_ptr<const node> m_parent;

            /**
             * @brief Constructs an uninitialized DijkstraNode.
             *
             * This constructor creates an DijkstraNode with uninitialized members.
             */
            [[nodiscard]] constexpr node() {} // NOLINT(*-pro-type-member-init, *-use-equals-default)

            [[nodiscard]] constexpr node(const index_t& _index, const scalar_t &_gScore, const scalar_t& _hScore, const std::shared_ptr<const node>& _parent) :
                m_index(_index),
                m_gScore(_gScore),
                m_fScore(_gScore + _hScore),
                m_parent(std::move(_parent)) {}

            ~node() { // NOLINT(*-use-equals-default)

//                while (m_parent && static_cast<unsigned>(m_parent.use_count()) < 2U) {
//                    m_parent = std::move(m_parent->m_parent);
//                }

                expungeRecursive(m_parent);
            }

            void expunge_recursive(std::shared_ptr<const node>& _node) {
                if (_node && _node.unique()) {
                    _node = std::move(_node->m_parent);
                    expungeRecursive(_node);
                }
            }

            [[nodiscard]] friend constexpr bool operator == (const node& _a, const node& _b) noexcept { return _a.m_index == _b.m_index; }

            [[nodiscard]] friend constexpr bool operator < (const node& _a, const node& _b) noexcept {
                return _a.m_fScore == _b.m_fScore ?
                       _a.m_gScore >  _b.m_gScore :
                       _a.m_fScore >  _b.m_fScore;
            }
        };

        [[maybe_unused, nodiscard]] static constexpr std::vector<coord_t> execute(const params_t& _params) {

            (void)_params;

            throw std::runtime_error("Djikstra::solve(const Mazes::Grid<Kd, weight_t>& _maze, const coord_t& _start, const coord_t& _end, scalar_t (*_h)(const coord_t&, const coord_t&), size_t _capacity = 0U): Not implemented!");
        }

    };

} //chdr::solvers

#endif //CHDR_DJIKSTRA_HPP