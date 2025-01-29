/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_GRAPH_HPP
#define CHDR_GRAPH_HPP

/**
 * @file graph.hpp
 */

#include <cassert>
#include <cstddef>
#include <future>
#include <initializer_list>
#include <iostream>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "../types/containers/stack.hpp"
#include "../utils/utils.hpp"
#include "grid.hpp"
#include "nodes/id_node.hpp"

namespace chdr::mazes {

    /**
     * @nosubgrouping
     * @class graph
     * @brief Graph structure for use in maze-solving algorithms.
     *
     * @details A mutable graph represented using an adjacency set.
     *          Supports adding and removing nodes and edges, pruning for optimisation,
     *          and other utilities necessary for graph-based pathfinding operations.
     *          
     * @tparam index_t  The type used for indexing nodes in the graph.
     * @tparam scalar_t The type used for edge weights in the graph.
     * 
     * @note This class uses polymorphic memory resources (`std::pmr::memory_resource`).
     * @note Follows an STL-like design and supports iterators.
     * 
     * @warning It is very strongly recommended that you use a pool allocation scheme
     *          (i.e `std::pmr::unsynchronized_pool_resource`, `std::pmr::synchronized_pool_resource`)
     *          with this data structure to improve data locality and reduce fragmentation.
     */
    template<typename index_t, typename scalar_t>
    class graph final {

    public:

        using       edge_t = std::pair<index_t, scalar_t>;
        using neighbours_t = std::pmr::vector<edge_t>;

    private:

        using adjacency_set_t = std::pmr::unordered_map<index_t, neighbours_t>;

        adjacency_set_t m_entries;

    public:

        using neighbour_t = edge_t;

        /**
         * @name Constructors
         * @{
         */

        /**
         * @brief Constructs a `graph` instance with default initialisation.
         *
         * @details Creates an empty graph.
         *
         * @param [in] _resource A pointer to a memory resource for allocator usage. Defaults to
         *                       `std::pmr::get_default_resource`. (optional)
         *
         * @return A `graph` instance with no nodes or edges.
         */
        [[maybe_unused]] constexpr graph(std::pmr::memory_resource* _resource = std::pmr::get_default_resource()) noexcept : m_entries(_resource) {}

        /**
         * @brief Constructs a `graph` instance from an adjacency list.
         *
         * @details Initialises a graph using an `std::initializer_list` that represents the adjacency list for the graph.
         *          Each inner list contains the edges associated with a single node, where each edge is represented as a pair of
         *          the destination node index and its weight. Optionally, a memory resource can be specified for resource management.
         *
         * @param[in] _adjacency_list  A two-dimensional `std::initializer_list` representing the adjacency list.
         *                            Each sublist corresponds to the edges of a single node.
         *
         * @param [in] _resource A pointer to a memory resource for allocator usage. Defaults to
         *                       `std::pmr::get_default_resource`. (optional)
         *
         * @return A `graph` instance with nodes and edges created from the provided adjacency list.
         */
        [[maybe_unused]] constexpr graph(const std::initializer_list<std::initializer_list<edge_t>>& _adjacency_list, std::pmr::memory_resource* _resource = std::pmr::get_default_resource()) : m_entries(_resource) {

            index_t index{0};

            for (const auto& entry: _adjacency_list) {

                for (const auto& edge: entry) {
                    add(index, edge);
                }

                ++index;
            }
        }

        /**
         * @brief Constructs a `graph` instance from a grid structure.
         *
         * @details Builds a graph representation of the provided grid, where each active grid cell is treated 
         *          as a node and connections to its neighbors are treated as edges.\n\n
         *          If pruning is enabled (default), intermediate transitory nodes are removed, and longer 
         *          direct connections are established to produce a more compact graph representation. 
         *          Multi-threading is used to improve performance during graph construction.
         *
         * @tparam coord_t The data type used for grid coordinates.
         * @tparam weight_t The data type used for edge weights.
         * @tparam Prune A boolean flag indicating whether transitory nodes should be pruned (default: true).
         * @param _grid The input grid structure from which the graph is constructed.
         * @param _resource A pointer to a memory resource for allocator usage. Defaults to
         *                  `std::pmr::get_default_resource`. (optional)
         */
        template <typename coord_t, typename weight_t, const bool Prune = true>
        [[maybe_unused]]
#if defined(__cpp_constexpr_dynamic_alloc) && (__cpp_constexpr_dynamic_alloc >= 201907L)
        constexpr
#endif // defined(__cpp_constexpr_dynamic_alloc) && (__cpp_constexpr_dynamic_alloc >= 201907L)
        explicit graph(const grid<coord_t, weight_t>& _grid, std::pmr::memory_resource* _resource = std::pmr::get_default_resource()) : m_entries(_resource) {

            const auto size = _grid.size();

            if constexpr (!Prune) {

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
            else {

                // TODO: Add support for directed graphs.
                std::mutex mtx;

                const auto worker = [&](index_t _start, index_t _end) ALWAYS_INLINE {

                    std::pmr::unsynchronized_pool_resource pool;
                    stack<edge_t> stack(&pool);

                    std::pmr::unordered_set<index_t> global_closed(&pool);
                    std::pmr::unordered_set<index_t> local_closed(&pool);

                    std::pmr::unordered_map<index_t, neighbours_t> thread_connections(&pool);

                    for (auto index = _start; index < _end; ++index) {

                        if (const auto& element = _grid.at(index); element.is_active()) {

                            global_closed.clear();
                            global_closed.insert(index);

                            const auto& neighbours = _grid.get_neighbours(index);

                            if (!_grid.is_transitory(neighbours)) {

                                for (const auto& n1 : neighbours) {

                                    if (const auto& [nActive1, nCoord1] = n1; nActive1) {

                                        const auto nIdx1 = static_cast<index_t>(utils::to_1d(nCoord1, size));

                                        local_closed.clear();
                                        stack.emplace(std::make_pair(nIdx1, static_cast<scalar_t>(1)));

                                        while (!stack.empty()) {

                                            const auto [currIdx, currDistance] = std::move(stack.top());
                                            stack.pop();

                                            if (local_closed.find(currIdx) == local_closed.end()) {

                                                 local_closed.insert(currIdx);
                                                global_closed.insert(currIdx);

                                                for (const auto& n3 : _grid.get_neighbours(currIdx)) {

                                                    if (const auto& [sActive3, sCoord3] = n3; sActive3) {

                                                        const auto s = static_cast<index_t>(utils::to_1d(sCoord3, size));

                                                        if (global_closed.find(s) == global_closed.end()) {

                                                            const edge_t next = std::make_pair(s, currDistance + static_cast<scalar_t>(1));

                                                            if (_grid.is_transitory(s)) {
                                                                stack.emplace(std::move(next));
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

                // Create a thread pool to perform the work in parallel:
                const size_t count      = _grid.count();
                const size_t numThreads = utils::clamp(std::thread::hardware_concurrency(), 1U, 6U);

                const auto chunkSize = static_cast<index_t>((count + numThreads - 1U) / numThreads);

                std::vector<std::future<void>> futures;
                futures.reserve(numThreads);

                for (size_t i = 0U; i < numThreads; ++i) {

                    const index_t start = static_cast<index_t>(i) * chunkSize;
                    const index_t end   = utils::min(start + chunkSize, static_cast<index_t>(count));

                    futures.emplace_back(std::async(std::launch::async, worker, start, end));
                }

                for (auto& fut : futures) {
                    fut.get();
                }

                chdr::malloc_consolidate();
            }
        }

        [[nodiscard]] constexpr graph           (const graph&) = delete;
                      constexpr graph& operator=(const graph&) = delete;

        [[nodiscard]] constexpr graph(graph&&) noexcept = default;

#if __cplusplus > 202302L
        constexpr
#endif
        graph& operator=(graph&&) noexcept = default;

        /**
         * @}
         */

        [[nodiscard]] constexpr const auto& at(const index_t& _id) const {

            assert(contains(_id) && "Error: The node with the specified ID does not exist in the graph.");
            return reinterpret_cast<const id_node<index_t>&>(_id);
        }

        constexpr void add(index_t _from_id) {
            m_entries.insert_or_assign(_from_id, neighbours_t{});
        }

        constexpr void add(index_t _from_id, const edge_t& _edge) {
            m_entries[_from_id].emplace_back(_edge);
        }

        [[maybe_unused]] constexpr void remove(index_t _from_id, const edge_t& _edge) noexcept {

            if (contains(_from_id)) {

                auto& bucket = m_entries[_from_id];
                bucket.erase(std::remove(bucket.begin(), bucket.end(), _edge), bucket.end());

                if (bucket.empty()) {
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

            chdr::malloc_consolidate();
        }

        [[maybe_unused]] void print() const noexcept {

            for (const auto& [node, edges]: m_entries) {

                std::cout << "Node " << node << ":\n";

                for (const auto& edge: edges) {
                    std::cout << "  -> (" << edge.first << ", " << edge.second << ")\n";
                }
            }
        }

        [[maybe_unused, nodiscard]] HOT constexpr const neighbours_t& get_neighbours(index_t _id) const {
            assert(contains(_id) && "Node with the specified ID does not exist in the graph.");
            return m_entries.find(_id)->second;
        }

        [[maybe_unused, nodiscard]] HOT constexpr bool contains(index_t _id) const noexcept {
            return m_entries.find(_id) != m_entries.end();
        }

        [[maybe_unused, nodiscard]] constexpr size_t count() const noexcept {
            return m_entries.size();
        }

        [[maybe_unused]] void clear() noexcept {
            m_entries.clear();
        }

        [[nodiscard]] HOT constexpr const id_node<index_t>& operator[](size_t _id) const noexcept {
            return at(_id);
        }

        using       iterator_t = typename adjacency_set_t::      iterator;
        using const_iterator_t = typename adjacency_set_t::const_iterator;

        [[maybe_unused, nodiscard]]       iterator_t  begin()       noexcept { return m_entries.begin();  }
        [[maybe_unused, nodiscard]] const_iterator_t  begin() const noexcept { return m_entries.begin();  }
        [[maybe_unused, nodiscard]] const_iterator_t cbegin() const noexcept { return m_entries.cbegin(); }

        [[maybe_unused, nodiscard]]       iterator_t  end()       noexcept { return m_entries.end();  }
        [[maybe_unused, nodiscard]] const_iterator_t  end() const noexcept { return m_entries.end();  }
        [[maybe_unused, nodiscard]] const_iterator_t cend() const noexcept { return m_entries.cend(); }

    };

} //chdr::mazes

#endif //CHDR_GRAPH_HPP