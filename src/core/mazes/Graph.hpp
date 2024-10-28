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
#include <stack>

namespace CHDR::Mazes {

    template<typename Ti, size_t Kd, typename Ts = float>
    class Graph : public IGraph<Ti, Ts> {

    private:

        using edge_t = typename IGraph<Ti, Ts>::edge_t;

        struct IndexHash {

            constexpr size_t operator () (const Ti& _index) const {
                return static_cast<Ti>(_index);
            }

        };

        struct IndexEqual {

            constexpr bool operator () (const Ti& _a, const Ti& _b) const {
                return _a == _b;
            }

        };

        struct EdgeHash {

            constexpr size_t operator () (const std::pair<Ti, Ts> &_edge) const {
                return static_cast<size_t>(_edge.first) ^ (std::hash<Ts>()(_edge.second) << 1);
            }
        };

        struct EdgeEqual {

            constexpr bool operator () (const std::pair<Ti, Ts>& _a, const std::pair<Ti, Ts>& _b) const {
                return _a.first == _b.first && _a.second == _b.second;
            }
        };

        using Neighbours    = std::unordered_set<edge_t, EdgeHash, EdgeEqual>;
        using AdjacencyList = std::unordered_map<Ti, Neighbours, IndexHash, IndexEqual>;

        AdjacencyList m_Entries;

    public:

        Graph(std::initializer_list<std::initializer_list<edge_t>> _adjacency_list) : m_Entries{} {

            size_t node_id = 0U;

            for (const auto& entry: _adjacency_list) {
                for (const auto& edge: entry) {
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

            const auto size = _grid.Size();

            Ti index{0};

            constexpr bool Sparse = true;
            if constexpr (Sparse) {

            const auto count = _grid.Size();

                const size_t _capacity = _grid.Count() / 4U;

                ExistenceSet<LowMemoryUsage> visited(_capacity);
                ExistenceSet<LowMemoryUsage> closed(_capacity);

                std::vector<std::pair<Ti, Ts>> nodesToVisit(64U);

                std::unordered_map<Ti, std::vector<std::pair<Ti, Ts>>> transitoryConnections;

                for (auto& element : _grid) {

                    if (element.IsActive()) {

                        closed.Clear();
                        if (closed.Capacity() < index) {
                            closed.Reserve(_capacity * ((index % _capacity) + 1U));
                        }
                        closed.Add(index);

                        // Check not transitory (fast, partial).
                        if (transitoryConnections.find(index) == transitoryConnections.end()) {

                            auto index_neighbours = _grid.GetNeighbours(index);

                            // Check not transitory (slow, exact).
                            size_t nCount = 0U;
                            for (const auto& n1 : index_neighbours) {
                                if (const auto& [nActive, nCoord] = n1; nActive && ++nCount > 2U) {
                                    break;
                                }
                            }

                            // If not transitory.
                            if (nCount != 2U) {

                                for (const auto& n1 : index_neighbours) {

                                    if (const auto& [nActive1, nCoord1] = n1; nActive1) {

                                        const auto nIdx1 = Utils::To1D(nCoord1, size);

                                        //auto t_search = transitoryConnections.find(nIdx1);
                                        //if (t_search == transitoryConnections.end()) {

                                            visited.Clear();
                                            nodesToVisit.push_back({ nIdx1, static_cast<Ts>(1) });

                                            while (!nodesToVisit.empty()) {

                                                auto [curr_idx, curr_distance] = std::move(nodesToVisit.back());
                                                nodesToVisit.pop_back();

                                                if (!visited.Contains(curr_idx)) {

                                                    if (visited.Capacity() < curr_idx) {
                                                        visited.Reserve(_capacity * ((curr_idx % _capacity) + 1U));
                                                    }
                                                    visited.Add(curr_idx);

                                                    if (closed.Capacity() < curr_idx) {
                                                        closed.Reserve(_capacity * ((curr_idx % _capacity) + 1U));
                                                    }
                                                    closed.Add(curr_idx);

                                                    for (const auto& n3 : _grid.GetNeighbours(curr_idx)) {

                                                        if (const auto& [sActive3, sCoord3] = n3; sActive3) {

                                                            const auto s = Utils::To1D(sCoord3, size);

                                                            if (!closed.Contains(s)) {

                                                                auto next = std::make_pair(s, curr_distance + static_cast<Ts>(1));

                                                                const bool isTransitory3 = transitoryConnections.find(s) != transitoryConnections.end() || _grid.IsTransitory(s);

                                                                if (isTransitory3) {
                                                                    nodesToVisit.push_back(next);
                                                                }
                                                                else {

                                                                    auto search = transitoryConnections.find(nIdx1);
                                                                    if (search != transitoryConnections.end()) {
                                                                        search->second.push_back(next);
                                                                    }
                                                                    else {
                                                                        transitoryConnections.insert({ nIdx1, { next } });
                                                                    }

                                                                    nodesToVisit.clear();
                                                                    break;
                                                                }
                                                            }
                                                        }
                                                    }
                                                }
                                            }
                                        //}

                                        if (auto t_search = transitoryConnections.find(nIdx1); t_search != transitoryConnections.end()) {
                                            for (const auto& item : t_search->second) {
                                                Add(index, item);
                                            }
                                        }
                                        else if (!closed.Contains(nIdx1)) {
                                            Add(index, { nIdx1, static_cast<Ts>(1) });
                                        }
                                    }
                                }
                            }
                        }
                    }
                    ++index;
                }
            }
            else {
                for (auto& element : _grid) {

                    if (element.IsActive()) {

                        for (auto& neighbour : _grid.GetNeighbours(index)) {

                            if (const auto& [nActive, nCoord] = neighbour; nActive) {

                                const auto n = Utils::To1D(nCoord, size);

                                Add(index, std::make_pair(n, static_cast<Ts>(1)));
                            }
                        }
                    }

                    ++index;
                }
            }
        }

        template<typename... Args>
        [[nodiscard]] constexpr IDNode<Ti> At(const Args&... _id) const {
            return At({ _id... });
        }

        [[nodiscard]] constexpr IDNode<Ti> At(const Ti& _id) const {

            auto search = m_Entries.find(_id);

#ifndef NDEBUG
            if (search == m_Entries.end()) {
                throw std::runtime_error("Error: The node with the specified ID does not exist in the graph.");
            }
#endif // NDEBUG

            return { search->first };
        }

        void Add(const Ti& _from_id, const edge_t& _edge) {
            m_Entries[_from_id].emplace(_edge);
        }

        void Remove(const Ti& _from_id, const edge_t& _edge) {

            if (Contains(_from_id)) {
                m_Entries[_from_id].erase(_edge);

                if (m_Entries[_from_id].empty()) {
                    m_Entries.erase(_from_id);
                }
            }
        }

        void Prune() {

            std::vector<Ti> nodesToRemove;

            do {
                nodesToRemove.clear();

                for (const auto& entry : m_Entries) {

                    auto& [node, neighbours] = entry;

                    if (m_Entries.size() <= 2U) {
                        break;
                    }
                    else {

                        if (neighbours.size() == 2U) {

                            auto& [n1_id, n1_cost] = *(  neighbours.begin());
                            auto& [n2_id, n2_cost] = *(++neighbours.begin());

                            auto s1 = m_Entries.find(n1_id);
                            auto s2 = m_Entries.find(n2_id);

                            // Merge the connections from the current node with its neighbours:
                            if (s1 != m_Entries.end()) {

                                auto& set = s1->second;
                                auto sn = set.find(std::make_pair(node, n1_cost));

                                if (sn != set.end()) {
                                    set.emplace(n2_id, sn->second + n2_cost);
                                    set.erase(sn);
                                }
                            }
                            if (s2 != m_Entries.end()) {

                                auto& set = s2->second;
                                auto sn = set.find(std::make_pair(node, n2_cost));

                                if (sn != set.end()) {
                                    set.emplace(n1_id, sn->second + n1_cost);
                                    set.erase(sn);
                                }
                            }

                            // Remove the current node from the graph.
                            nodesToRemove.emplace_back(node);
                        }
                    }
                }
                for (const auto& node : nodesToRemove) {
                    m_Entries.erase(node);
                }
            }
            while (!nodesToRemove.empty());

            __malloc_consolidate();
        }

        void Print() const {

            for (const auto& [node, edges]: m_Entries) {

                std::cout << "Node " << node << ":\n";

                for (const auto& edge: edges) {
                    std::cout << "  -> (" << edge.first << ", " << edge.second << ")\n";
                }
            }
        }

        [[nodiscard]] constexpr const Neighbours& GetNeighbours(const Ti& _id) const {
            return m_Entries.find(_id)->second;
        }

        [[nodiscard]] constexpr bool Contains(const Ti& _id) const override {
            return m_Entries.find(_id) != m_Entries.end();
        }

        [[nodiscard]] constexpr size_t Count() const override {
            return m_Entries.size();
        }

        void Clear() {
            m_Entries.clear();
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