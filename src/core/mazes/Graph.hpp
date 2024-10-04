/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_GRAPH_HPP
#define CHDR_GRAPH_HPP

#include "base/IMaze.hpp"

#include "../nodes/IDNode.hpp"

namespace CHDR::Mazes {

    template <typename Ti, typename Ts = float>
    class Graph : public IMaze<IDNode<Ti>> {

        using edge_t = std::pair<Ti, Ts>;

    private:

        std::vector<IDNode<Ti>> m_Nodes;
        std::vector<std::vector<edge_t>> m_Edges;

    public:

        void AddNode(const std::initializer_list<IDNode<Ti>> &_initialiser) {
            m_Nodes.emplace_back(_initialiser).ID(m_Nodes.size() - 1U);
        }

        void AddEdge(const IDNode<Ti>& _from, const IDNode<Ti>& _to, const Ts& _distance) {
            m_Edges[_from].emplace_back(_to, _distance);
        }

        [[nodiscard]] constexpr edge_t GetNeighbours(const Ti _id) {
            return m_Edges[_id];
        }

        [[nodiscard]] constexpr const IDNode<Ti>& At(const Ti &_id) const override {
            return m_Nodes[_id];
        };

        [[nodiscard]] constexpr bool Contains(const Ti &_id) const override {
            return _id < m_Nodes.size();
        }

    };

} // CHDR::Mazes

#endif //CHDR_GRAPH_HPP