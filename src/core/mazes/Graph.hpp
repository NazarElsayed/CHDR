/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_GRAPH_HPP
#define CHDR_GRAPH_HPP

#include "base/IGraph.hpp"
#include "Grid.hpp"

#include "base/IGraph.hpp"
#include "types/ExistenceSet.hpp"

#include <functional>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace CHDR::Mazes {

    template<typename Ti, size_t Kd, typename Ts = float>
    class Graph : public IGraph<Ti, Ts> {

    private:

        using edge_t = typename IGraph<Ti, Ts>::edge_t;

        struct EdgeHash {

            std::size_t operator()(const std::pair<Ti, Ts> &edge) const {

                std::size_t h1 = std::hash<Ti>()(edge.first);
                std::size_t h2 = std::hash<Ts>()(edge.second);

                return h1 ^ (h2 << 1);
            }
        };

        struct EdgeEqual {

            bool operator()(const std::pair<Ti, Ts> &edge1, const std::pair<Ti, Ts> &edge2) const {
                return edge1.first == edge2.first && edge1.second == edge2.second;
            }
        };

        using AdjacencyList = std::unordered_map<Ti, std::unordered_set<edge_t, EdgeHash, EdgeEqual>>;

        AdjacencyList m_Entries;

    public:

        Graph(std::initializer_list<std::initializer_list<edge_t>> _adjacency_list) : m_Entries{} {

            size_t node_id = 0U;

            for (const auto &entry: _adjacency_list) {
                for (const auto &edge: entry) {
                    Add(node_id, edge);
                }
                ++node_id;
            }
        }

        template <typename Tm>
#if __cplusplus >= 202302L
        constexpr
#endif // __cplusplus >= 202302L
        Graph(const Grid<Kd, Tm>& _grid) : m_Entries{} {

            /* Convert grid to (dense) graph. */

            Ti index{0};
            for (auto& element : _grid) {

                if (element.IsActive()) {

                    for (auto& neighbour : _grid.GetNeighbours(index)) {

                        if (const auto& [nActive, nCoord] = neighbour; nActive) {

                            const auto n = Utils::To1D(nCoord, _grid.Size());

                            Add(index, std::make_pair(n, static_cast<Ts>(1.0)));
                        }
                    }
                }

                ++index;
            }
        }

        void Add(const Ti& _from_id, const edge_t& _edge) {
            m_Entries[_from_id].insert(_edge);
        }

        void Remove(const Ti& _from_id, const edge_t& _edge) {

            if (Contains(_from_id)) {
                m_Entries[_from_id].erase(_edge);

                if (m_Entries[_from_id].empty()) {
                    m_Entries.erase(_from_id);
                }
            }
        }

        void MakeSparse() {

            for (size_t i = 0U; i < 1U; ++i) {

                std::vector<std::pair<Ti, edge_t>> edges_to_remove;
                std::vector<std::pair<Ti, edge_t>> edges_to_add;

                for (const auto &[current, outgoing_edges]: m_Entries) { // FOR EACH NODE IN THE GRAPH ...
                    for (const auto &outgoing_edge: outgoing_edges) {
                        const auto &[other, dOut] = outgoing_edge;

                        for (const auto &[neighbour, dIn]: m_Entries.at(other)) {
                            if (neighbour == current) { // GET EVERY CONNECTION WHICH CONNECTS TO IT ...

                                // Identify those connections which are transitory.
                                const bool isTransitory = dIn == dOut;

                                if (isTransitory) {
                                    edges_to_remove.emplace_back(current, outgoing_edge);
                                    edges_to_add.emplace_back(other, std::make_pair(current, dOut + dIn));
                                }
                            }
                        }
                    }
                }

                for (const auto &[node, edge]: edges_to_remove) {
                    Remove(node, edge);
                }

                for (const auto &[node, edge]: edges_to_add) {
                    Add(node, edge);
                }
            }

        }

        void Print() const {
            for (const auto &[node, edges]: m_Entries) {
                std::cout << "Node " << node << ":\n";
                for (const auto &edge: edges) {
                    std::cout << "  -> (" << edge.first << ", " << edge.second << ")\n";
                }
            }
        }

        [[nodiscard]] constexpr const std::unordered_set<edge_t, EdgeHash, EdgeEqual>& GetNeighbours(const Ti &_id) const {

            auto it = m_Entries.find(_id);
            return it->second;
        }

        [[nodiscard]] constexpr bool Contains(const Ti& _id) const override {
            return m_Entries.find(_id) != m_Entries.end();
        }

        [[nodiscard]] constexpr size_t Count() const override {
            return m_Entries.size();
        }

        using       iterator = typename std::unordered_map<Ti, std::vector<edge_t>>::iterator;
        using const_iterator = typename std::unordered_map<Ti, std::vector<edge_t>>::const_iterator;

              iterator  begin()       { return m_Entries.begin();  }
        const_iterator  begin() const { return m_Entries.begin();  }
        const_iterator cbegin() const { return m_Entries.cbegin(); }

              iterator  end()       { return m_Entries.end();  }
        const_iterator  end() const { return m_Entries.end();  }
        const_iterator cend() const { return m_Entries.cend(); }

    };

} // CHDR::Mazes

#endif //CHDR_GRAPH_HPP