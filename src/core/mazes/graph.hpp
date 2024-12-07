/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_GRAPH_HPP
#define CHDR_GRAPH_HPP

#include <functional>
#include <future>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "grid.hpp"
#include "base/igraph.hpp"
#include "utils/intrinsics.hpp"

namespace chdr::mazes {

    template<typename index_t, typename scalar_t>
    class graph final : public igraph<index_t, scalar_t> {

    private:

        using edge_t = typename igraph<index_t, scalar_t>::edge_t;

        struct index_hash {

            constexpr size_t operator () (const index_t& _index) const {
                return static_cast<index_t>(_index);
            }
        };

        struct index_equal {

            constexpr bool operator () (const index_t& _a, const index_t& _b) const {
                return _a == _b;
            }
        };

        struct edge_hash {

            constexpr size_t operator () (const std::pair<index_t, scalar_t> &_edge) const {
                return static_cast<size_t>(_edge.first) ^ (std::hash<scalar_t>()(_edge.second) << 1);
            }
        };

        struct edge_equal {

            constexpr bool operator () (const std::pair<index_t, scalar_t>& _a, const std::pair<index_t, scalar_t>& _b) const {
                return _a.first == _b.first && _a.second == _b.second;
            }
        };

        using neighbours_t    = std::unordered_set<edge_t, edge_hash, edge_equal>;
        using adjacency_set_t = std::unordered_map<index_t, neighbours_t, index_hash, index_equal>;

        adjacency_set_t m_entries;

    public:

        [[maybe_unused]] constexpr graph() : m_entries() {}

        [[maybe_unused]] constexpr graph(std::initializer_list<std::initializer_list<edge_t>> _adjacencyList) : m_entries() {

            index_t index{0};

            for (const auto& entry: _adjacencyList) {

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
        [[maybe_unused]] explicit graph(const grid<Kd, weight_t>& _grid, const constexpr bool& _prune = true) : m_entries{} {

            const auto size = _grid.size();

            if constexpr (_prune) {

                // TODO: Add support for directed graphs.

                std::mutex mtx;

                auto worker = [&](const index_t& _start, const index_t& _end) {

                    std::vector<edge_t> stack(128U);

                    std::unordered_set<index_t, index_hash, index_equal> global_closed;
                    std::unordered_set<index_t, index_hash, index_equal> local_closed;

                    std::unordered_map<index_t, std::vector<edge_t>, index_hash, index_equal> thread_connections;

                    for (auto index = _start; index < _end; ++index) {
                        
                        if (const auto& element = _grid.at(index); element.is_active()) {

                            global_closed.clear();
                            global_closed.insert(index);

                            auto index_neighbours = _grid.get_neighbours(index);

                            size_t nCount = 0U;
                            for (const auto& n1 : index_neighbours) {
                                if (const auto& [nActive, nCoord] = n1; nActive && ++nCount > 2U) {
                                    break;
                                }
                            }

                            if (nCount != 2U) {

                                for (const auto& n1 : index_neighbours) {

                                    if (const auto& [nActive1, nCoord1] = n1; nActive1) {

                                        const auto nIdx1 = utils::to_1d(nCoord1, size);

                                        local_closed.clear();
                                        stack.emplace_back(std::make_pair(nIdx1, static_cast<scalar_t>(1)));

                                        while (!stack.empty()) {

                                            const auto [currIdx, currDistance] = std::move(stack.back());
                                            stack.pop_back();

                                            if (local_closed.find(currIdx) == local_closed.end()) {

                                                 local_closed.insert(currIdx);
                                                global_closed.insert(currIdx);

                                                for (const auto& n3 : _grid.get_neighbours(currIdx)) {

                                                    if (const auto& [sActive3, sCoord3] = n3; sActive3) {

                                                        const auto s = utils::to_1d(sCoord3, size);

                                                        if (global_closed.find(s) == global_closed.end()) {

                                                            const auto next = std::make_pair(s, currDistance + static_cast<scalar_t>(1));

                                                            if (_grid.is_transitory(s)) {
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
                                            const std::lock_guard lock(mtx);
                                            add(index, item);
                                        }
                                    }
                                }
                            }
                        }
                    }
                };

                // Create a thread pool to execute the work in parallel:
                const size_t count = _grid.count();
                const size_t numThreads = std::clamp(std::thread::hardware_concurrency(), 1U, 6U);

                const auto chunkSize = static_cast<index_t>((count + numThreads - 1U) / numThreads);

                std::vector<std::future<void>> futures;
                for (size_t i = 0U; i < numThreads; ++i) {

                    const index_t start = i * chunkSize;
                    const index_t end   = std::min(start + chunkSize, static_cast<index_t>(count));

                    futures.push_back(std::async(std::launch::async, worker, start, end));
                }

                for (auto& fut : futures) {
                    fut.get();
                }
            }
            else {

                index_t index{0};
                for (auto& element : _grid) {

                    if (element.is_active()) {

                        for (auto& neighbour : _grid.get_neighbours(index)) {

                            if (const auto& [nActive, nCoord] = neighbour; nActive) {
                                add(index, std::make_pair(utils::to_1d(nCoord, size), static_cast<scalar_t>(1)));
                            }
                        }
                    }

                    ++index;
                }
            }

            malloc_consolidate();
        }

        template<typename... Args>
        [[nodiscard]] constexpr id_node<index_t> at(const Args&... _id) const {
            return at({_id...});
        }

        [[nodiscard]] constexpr id_node<index_t> at(const index_t& _id) const {

            auto search = m_entries.find(_id);

#ifndef NDEBUG
            if (search == m_entries.end()) {
                throw std::runtime_error("Error: The node with the specified ID does not exist in the graph.");
            }
#endif // NDEBUG

            return { search->first };
        }

        void add(const index_t& _from_id) {
            m_entries.insert_or_assign(_from_id, neighbours_t{});
        }

        void add(const index_t& _from_id, const edge_t& _edge) {
            m_entries[_from_id].emplace(_edge);
        }

        [[maybe_unused]] void remove(const index_t& _from_id, const edge_t& _edge) {

            if (contains(_from_id)) {
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

        [[maybe_unused]] [[nodiscard]] constexpr const neighbours_t& get_neighbours(const index_t& _id) const {
        
#ifndef NDEBUG
            if (!contains(_id)) {
              throw std::runtime_error("Node with the specified ID does not exist in the graph.");
            }
#endif

            return m_entries.find(_id)->second;
        }

        [[maybe_unused]] [[nodiscard]] constexpr bool contains(const index_t& _id) const override {
            return m_entries.find(_id) != m_entries.end();
        }

        [[maybe_unused]] [[nodiscard]] constexpr size_t count() const override {
            return m_entries.size();
        }

        [[maybe_unused]] void clear() {
            m_entries.clear();
        }

        using       iterator_t = typename adjacency_set_t::iterator;
        using const_iterator_t = typename adjacency_set_t::const_iterator;

        [[maybe_unused]] [[nodiscard]]       iterator_t  begin()       { return m_entries.begin();  }
        [[maybe_unused]] [[nodiscard]] const_iterator_t  begin() const { return m_entries.begin();  }
        [[maybe_unused]] [[nodiscard]] const_iterator_t cbegin() const { return m_entries.cbegin(); }

        [[maybe_unused]] [[nodiscard]]       iterator_t  end()       { return m_entries.end();  }
        [[maybe_unused]] [[nodiscard]] const_iterator_t  end() const { return m_entries.end();  }
        [[maybe_unused]] [[nodiscard]] const_iterator_t cend() const { return m_entries.cend(); }

    };

} //chdr::mazes

#endif //CHDR_GRAPH_HPP