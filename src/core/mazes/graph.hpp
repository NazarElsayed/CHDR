/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_GRAPH_HPP
#define CHDR_GRAPH_HPP

#include "base/igraph.hpp"
#include "Grid.hpp"

#include "base/igraph.hpp"
#include "types/existence_set.hpp"

#include <atomic>
#include <functional>
#include <future>
#include <mutex>
#include <queue>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace chdr::mazes {

    template<typename index_t, typename scalar_t>
    class graph final : public igraph<index_t, scalar_t> {

    private:

        using edge_t = typename igraph<index_t, scalar_t>::edge_t;

        struct indexHash {

            constexpr size_t operator () (const index_t& _index) const {
                return static_cast<index_t>(_index);
            }
        };

        struct indexEqual {

            constexpr bool operator () (const index_t& _a, const index_t& _b) const {
                return _a == _b;
            }
        };

        struct edgeHash {

            constexpr size_t operator () (const std::pair<index_t, scalar_t> &_edge) const {
                return static_cast<size_t>(_edge.first) ^ (std::hash<scalar_t>()(_edge.second) << 1);
            }
        };

        struct edgeEqual {

            constexpr bool operator () (const std::pair<index_t, scalar_t>& _a, const std::pair<index_t, scalar_t>& _b) const {
                return _a.first == _b.first && _a.second == _b.second;
            }
        };

        using Neighbours   = std::unordered_set<edge_t, edgeHash, edgeEqual>;
        using AdjacencySet = std::unordered_map<index_t, Neighbours, indexHash, indexEqual>;

        AdjacencySet m_entries;

    public:

        [[maybe_unused]] constexpr graph() : m_entries() {}

        [[maybe_unused]] constexpr graph(std::initializer_list<std::initializer_list<edge_t>> _adjacency_list) : m_entries() {

            index_t index{0};

            for (const auto& entry: _adjacency_list) {

                for (const auto& edge: entry) {
                    add(index, edge);
                }

                ++index;
            }
        }

        template <size_t Kd, typename weight_t>
#if __cplusplus >= 202302L
        constexpr
#endif // __cplusplus >= 202302L
        [[maybe_unused]] explicit graph(const Grid<Kd, weight_t>& _grid) : m_entries{} {

            const auto size = _grid.Size();

            constexpr bool Prune = true;
            if constexpr (Prune) {

                // TODO: Add support for directed graphs.

                std::mutex mtx;
                std::atomic<index_t> index{0};

                auto worker = [&](const index_t& _start, const index_t& _end) {

                    std::vector<edge_t> stack(128U);

                    std::unordered_set<index_t, indexHash, indexEqual> global_closed;
                    std::unordered_set<index_t, indexHash, indexEqual> local_closed;

                    std::unordered_map<index_t, std::vector<edge_t>, indexHash, indexEqual> thread_connections;

                    for (index_t index = _start; index < _end; ++index) {

                        const auto& element = _grid.At(index);

                        if (element.IsActive()) {

                            global_closed.clear();
                            global_closed.insert(index);

                            auto index_neighbours = _grid.GetNeighbours(index);

                            size_t nCount = 0U;
                            for (const auto& n1 : index_neighbours) {
                                if (const auto& [nActive, nCoord] = n1; nActive && ++nCount > 2U) {
                                    break;
                                }
                            }

                            if (nCount != 2U) {

                                for (const auto& n1 : index_neighbours) {

                                    if (const auto& [nActive1, nCoord1] = n1; nActive1) {

                                        const auto nIdx1 = Utils::To1D(nCoord1, size);

                                        local_closed.clear();
                                        stack.emplace_back(std::make_pair(nIdx1, static_cast<scalar_t>(1)));

                                        while (!stack.empty()) {

                                            const auto [curr_idx, curr_distance] = std::move(stack.back());
                                            stack.pop_back();

                                            if (local_closed.find(curr_idx) == local_closed.end()) {

                                                local_closed.insert(curr_idx);
                                                global_closed.insert(curr_idx);

                                                for (const auto& n3 : _grid.GetNeighbours(curr_idx)) {

                                                    if (const auto& [sActive3, sCoord3] = n3; sActive3) {

                                                        const auto s = Utils::To1D(sCoord3, size);

                                                        if (global_closed.find(s) == global_closed.end()) {

                                                            const auto next = std::make_pair(s, curr_distance + static_cast<scalar_t>(1));

                                                            if (_grid.IsTransitory(s)) {
                                                                stack.emplace_back(std::move(next));
                                                            }
                                                            else {
                                                                thread_connections[nIdx1].emplace_back(std::move(next));
                                                                stack.clear();
                                                                break;
                                                            }
                                                        }
                                                    }
                                                }
                                            }
                                        }

                                        for (const auto& item : thread_connections[nIdx1]) {
                                            std::lock_guard<std::mutex> lock(mtx);
                                            add(index, item);
                                        }
                                    }
                                }
                            }
                        }
                    }
                };

                // Create a thread pool to execute the work in parallel:
                const size_t count = _grid.Count();
                const size_t num_threads = std::clamp(std::thread::hardware_concurrency(), 1U, 6U);

                const auto chunk_size = static_cast<index_t>((count + num_threads - 1U) / num_threads);

                std::vector<std::future<void>> futures;
                for (size_t i = 0U; i < num_threads; ++i) {

                    const index_t start = i * chunk_size;
                    const index_t end   = std::min(start + chunk_size, static_cast<index_t>(count));

                    futures.push_back(std::async(std::launch::async, worker, start, end));
                }

                for (auto& fut : futures) {
                    fut.get();
                }
            }
            else {

                index_t index{0};
                for (auto& element : _grid) {

                    if (element.IsActive()) {

                        for (auto& neighbour : _grid.GetNeighbours(index)) {

                            if (const auto& [nActive, nCoord] = neighbour; nActive) {
                                add(index, std::make_pair(Utils::To1D(nCoord, size), static_cast<scalar_t>(1)));
                            }
                        }
                    }

                    ++index;
                }
            }

            malloc_consolidate();
        }

        template<typename... Args>
        [[nodiscard]] constexpr IDNode<index_t> at(const Args&... _id) const {
            return at({_id...});
        }

        [[nodiscard]] constexpr IDNode<index_t> at(const index_t& _id) const {

            auto search = m_entries.find(_id);

#ifndef NDEBUG
            if (search == m_entries.end()) {
                throw std::runtime_error("Error: The node with the specified ID does not exist in the graph.");
            }
#endif // NDEBUG

            return { search->first };
        }

        void add(const index_t& _from_id) {
            m_entries.insert_or_assign(_from_id, Neighbours{});
        }

        void add(const index_t& _from_id, const edge_t& _edge) {
            m_entries[_from_id].emplace(_edge);
        }

        [[maybe_unused]] void remove(const index_t& _from_id, const edge_t& _edge) {

            if (Contains(_from_id)) {
                m_entries[_from_id].erase(_edge);

                if (m_entries[_from_id].empty()) {
                    m_entries.erase(_from_id);
                }
            }
        }

        [[maybe_unused]] void prune() {

            // TODO: Fix erroneous graph when pruned twice and add support for pruning directed graphs.

            std::vector<index_t> nodesToRemove;

            do {
                nodesToRemove.clear();

                for (const auto& entry : m_entries) {

                    auto& [node, neighbours] = entry;

                    if (m_entries.size() > 2U) {

                        if (neighbours.size() == 2U) {

                            auto& [n1_id, n1_cost] = *(  neighbours.begin());
                            auto& [n2_id, n2_cost] = *(++neighbours.begin());

                            auto s1 = m_entries.find(n1_id);
                            auto s2 = m_entries.find(n2_id);

                            // Merge the connections from the current node with its neighbours:
                            if (s1 != m_entries.end()) {

                                auto& set = s1->second;
                                auto sn = set.find(std::make_pair(node, n1_cost));

                                if (sn != set.end()) {
                                    set.emplace(n2_id, sn->second + n2_cost);
                                    set.erase(sn);
                                }
                            }
                            if (s2 != m_entries.end()) {

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
                    else {
                        break;
                    }
                }
                for (const auto& node : nodesToRemove) {
                    m_entries.erase(node);
                }
            }
            while (!nodesToRemove.empty());

            malloc_consolidate();
        }

        [[maybe_unused]] void print() const {

            for (const auto& [node, edges]: m_entries) {

                std::cout << "Node " << node << ":\n";

                for (const auto& edge: edges) {
                    std::cout << "  -> (" << edge.first << ", " << edge.second << ")\n";
                }
            }
        }

        [[maybe_unused]] [[nodiscard]] constexpr const Neighbours& GetNeighbours(const index_t& _id) const {
        
#ifndef NDEBUG
            if (!Contains(_id)) {
              throw std::runtime_error("Node with the specified ID does not exist in the graph.");
            }
#endif

            return m_entries.find(_id)->second;
        }

        [[maybe_unused]] [[nodiscard]] constexpr bool Contains(const index_t& _id) const override {
            return m_entries.find(_id) != m_entries.end();
        }

        [[maybe_unused]] [[nodiscard]] constexpr size_t Count() const override {
            return m_entries.size();
        }

        [[maybe_unused]] void Clear() {
            m_entries.clear();
        }

        using       iterator = typename AdjacencySet::iterator;
        using const_iterator = typename AdjacencySet::const_iterator;

        [[maybe_unused]] [[nodiscard]]       iterator  begin()       { return m_entries.begin();  }
        [[maybe_unused]] [[nodiscard]] const_iterator  begin() const { return m_entries.begin();  }
        [[maybe_unused]] [[nodiscard]] const_iterator cbegin() const { return m_entries.cbegin(); }

        [[maybe_unused]] [[nodiscard]]       iterator  end()       { return m_entries.end();  }
        [[maybe_unused]] [[nodiscard]] const_iterator  end() const { return m_entries.end();  }
        [[maybe_unused]] [[nodiscard]] const_iterator cend() const { return m_entries.cend(); }

    };

} // CHDR::Mazes

#endif //CHDR_GRAPH_HPP