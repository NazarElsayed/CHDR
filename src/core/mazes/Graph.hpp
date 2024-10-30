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

#include <atomic>
#include <functional>
#include <future>
#include <mutex>
#include <queue>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace CHDR::Mazes {

    template<typename Ti, typename Ts>
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

        using Neighbours   = std::unordered_set<edge_t, EdgeHash, EdgeEqual>;
        using AdjacencySet = std::unordered_map<Ti, Neighbours, IndexHash, IndexEqual>;

        AdjacencySet m_Entries;

    public:

        [[maybe_unused]] constexpr Graph() : m_Entries() {}

        [[maybe_unused]] constexpr Graph(std::initializer_list<std::initializer_list<edge_t>> _adjacency_list) : m_Entries() {

            Ti index{0};

            for (const auto& entry: _adjacency_list) {

                for (const auto& edge: entry) {
                    Add(index, edge);
                }

                ++index;
            }
        }

        template <size_t Kd, typename Tm>
#if __cplusplus >= 202302L
        constexpr
#endif // __cplusplus >= 202302L
        [[maybe_unused]] explicit Graph(const Grid<Kd, Tm>& _grid) : m_Entries{} {

            const auto size = _grid.Size();

            constexpr bool Prune = true;
            if constexpr (Prune) {

                std::mutex mtx;
                std::atomic<Ti> index{0};

                auto worker = [&](const Ti& _start, const Ti& _end) {

                    std::vector<edge_t> stack(128U);

                    std::unordered_set<Ti, IndexHash, IndexEqual> global_closed;
                    std::unordered_set<Ti, IndexHash, IndexEqual> local_closed;

                    std::unordered_map<Ti, std::vector<edge_t>, IndexHash, IndexEqual> thread_connections;

                    for (Ti index = _start; index < _end; ++index) {

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
                                        stack.emplace_back(std::make_pair(nIdx1, static_cast<Ts>(1)));

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

                                                            const auto next = std::make_pair(s, curr_distance + static_cast<Ts>(1));

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
                                            Add(index, item);
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

                const auto chunk_size = static_cast<Ti>((count + num_threads - 1U) / num_threads);

                std::vector<std::future<void>> futures;
                for (size_t i = 0U; i < num_threads; ++i) {

                    const Ti start = i * chunk_size;
                    const Ti end   = std::min(start + chunk_size, static_cast<Ti>(count));

                    futures.push_back(std::async(std::launch::async, worker, start, end));
                }

                for (auto& fut : futures) {
                    fut.get();
                }
            }
            else {

                Ti index{0};
                for (auto& element : _grid) {

                    if (element.IsActive()) {

                        for (auto& neighbour : _grid.GetNeighbours(index)) {

                            if (const auto& [nActive, nCoord] = neighbour; nActive) {
                                Add(index, std::make_pair(Utils::To1D(nCoord, size), static_cast<Ts>(1)));
                            }
                        }
                    }

                    ++index;
                }
            }

            malloc_consolidate();
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

        [[maybe_unused]] void Remove(const Ti& _from_id, const edge_t& _edge) {

            if (Contains(_from_id)) {
                m_Entries[_from_id].erase(_edge);

                if (m_Entries[_from_id].empty()) {
                    m_Entries.erase(_from_id);
                }
            }
        }

        [[maybe_unused]] void Prune() {

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

            malloc_consolidate();
        }

        [[maybe_unused]] void Print() const {

            for (const auto& [node, edges]: m_Entries) {

                std::cout << "Node " << node << ":\n";

                for (const auto& edge: edges) {
                    std::cout << "  -> (" << edge.first << ", " << edge.second << ")\n";
                }
            }
        }

        [[maybe_unused]] [[nodiscard]] constexpr const Neighbours& GetNeighbours(const Ti& _id) const {
            return m_Entries.find(_id)->second;
        }

        [[maybe_unused]] [[nodiscard]] constexpr bool Contains(const Ti& _id) const override {
            return m_Entries.find(_id) != m_Entries.end();
        }

        [[maybe_unused]] [[nodiscard]] constexpr size_t Count() const override {
            return m_Entries.size();
        }

        [[maybe_unused]] void Clear() {
            m_Entries.clear();
        }

        using       iterator = typename AdjacencySet::iterator;
        using const_iterator = typename AdjacencySet::const_iterator;

        [[maybe_unused]] [[nodiscard]]       iterator  begin()       { return m_Entries.begin();  }
        [[maybe_unused]] [[nodiscard]] const_iterator  begin() const { return m_Entries.begin();  }
        [[maybe_unused]] [[nodiscard]] const_iterator cbegin() const { return m_Entries.cbegin(); }

        [[maybe_unused]] [[nodiscard]]       iterator  end()       { return m_Entries.end();  }
        [[maybe_unused]] [[nodiscard]] const_iterator  end() const { return m_Entries.end();  }
        [[maybe_unused]] [[nodiscard]] const_iterator cend() const { return m_Entries.cend(); }

    };

} // CHDR::Mazes

#endif //CHDR_GRAPH_HPP