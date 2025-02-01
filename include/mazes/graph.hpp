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

#include <array>
#include <cassert>
#include <cstddef>
#include <future>
#include <initializer_list>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

// ReSharper disable once CppUnusedIncludeDirective
#include <memory_resource> // NOLINT(*-include-cleaner)

#include "grid.hpp"
#include "../types/containers/stack.hpp"
#include "../utils/utils.hpp"
#include "nodes/id_node.hpp"

// ReSharper disable once CppUnusedIncludeDirective
#include "../utils/intrinsics.hpp" // NOLINT(*-include-cleaner)

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
     *          (i.e. `std::pmr::unsynchronized_pool_resource`, `std::pmr::synchronized_pool_resource`)
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
         */
        [[maybe_unused]] constexpr graph(const std::initializer_list<std::initializer_list<edge_t>>& _adjacency_list, std::pmr::memory_resource* _resource = std::pmr::get_default_resource()) : m_entries(_resource) {

            index_t index {0};

            for (const auto& entry : _adjacency_list) {

                for (const auto& edge : entry) {
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
         *          If pruning is enabled, intermediate transitory nodes are removed, and longer
         *          direct connections are established to produce a more compact graph representation. 
         *          Multi-threading is used to improve performance during pruning.
         *
         * @param _grid The input grid structure from which the graph is constructed.
         * @param _resource A pointer to a memory resource for allocator usage. Defaults to
         *                  `std::pmr::get_default_resource`. (optional)
         *
         * @tparam coord_t The data type used for grid coordinates.
         * @tparam weight_t The data type used for edge weights.
         * @tparam Prune Indicates whether transitory nodes should be pruned (optional, default `true`).
         * @tparam ConsolidateAfterPrune If true, attempts to consolidate the managed heap after pruning (optional, default `true`).
         */
        template <typename coord_t, typename weight_t, const bool Prune = true, const bool ConsolidateAfterPrune = true>
        [[maybe_unused]]
#if defined(__cpp_constexpr_dynamic_alloc) && (__cpp_constexpr_dynamic_alloc >= 201907L)
        constexpr
#endif // defined(__cpp_constexpr_dynamic_alloc) && (__cpp_constexpr_dynamic_alloc >= 201907L)
        explicit graph(const grid<coord_t, weight_t>& _grid, std::pmr::memory_resource* _resource = std::pmr::get_default_resource()) : m_entries(_resource) {

            const auto size = _grid.size();

            if constexpr (!Prune) {

                index_t index {0};
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

                // Create a thread pool to perform work in parallel:
                const size_t count      = _grid.count();
                const size_t numThreads = utils::clamp(std::thread::hardware_concurrency(), 1U, 6U);

                const auto chunkSize = static_cast<index_t>((count + numThreads - 1U) / numThreads);

                std::array<std::future<void>, 6U> futures;

                for (size_t i = 0U; i < numThreads; ++i) {
                    assert(i < numThreads && "Error: Index 'i' exceeds the number of initialised threads.");

                    const index_t start = static_cast<index_t>(i) * chunkSize;
                    const index_t end   = utils::min(start + chunkSize, static_cast<index_t>(count));

                    futures[i] = std::async(std::launch::async, worker, start, end);
                }

                for (size_t i = 0U; i < numThreads; ++i) {
                    assert(i < numThreads && "Error: Index 'i' exceeds the number of initialised threads.");

                    futures[i].get();
                }

                // Finalise the pruning process on a single thread.
                prune<ConsolidateAfterPrune>();
            }
        }

        ~graph() = default;

        [[nodiscard]] constexpr graph           (const graph&) = delete;
                      constexpr graph& operator=(const graph&) = delete;

        [[nodiscard]] constexpr graph(graph&&) noexcept = default;

#if __cplusplus > 202302L
        constexpr
#endif
        graph& operator=(graph&&) noexcept = default;

        /**
         * @brief Retrieves the vertex at a specified index.
         * @param _id Index of the vertex to retrieve.
         * @warning If the specified index is out of bounds, calling this function is undefined behaviour.
         * @return Vertex at a specified index within the grid.
         * @see contains()
         * @see operator[]()
         */
        [[nodiscard]] static constexpr const auto& at(const index_t& _id) {
            assert(contains(_id) && "Error: The node with the specified ID does not exist in the graph.");
            return reinterpret_cast<const id_node<index_t>&>(_id);
        }

        /**
         * @brief Adds a new vertex to the graph with the specified identifier.
         *
         * @details This method ensures that a vertex with the given identifier is added to
         *          the graph, if it does not already exist.
         *
         * @param [in] _from_id The identifier of the node to be added.
         */
        constexpr void add(index_t _from_id) {
            m_entries.try_emplace(_from_id, neighbours_t{});
        }

        /**
         * @brief Adds a new edge to the graph with the specified identifier.
         *
         * @details This method ensures that the specified edge is added to the graph. 
         *          If the given vertex does not exist in the graph, a new vertex with the
         *          specified identifier is created, and the edge is then added. 
         *          The edge is comprised of a destination node identifier and an associated weight.
         *          This operation always inserts the edge, even if an identical edge already exists.
         *
         * @param [in] _from_id The identifier of the node from which the edge originates.
         * @param [in] _edge    The edge to be added, represented as a pair of destination node and weight.
         */
        constexpr void add(index_t _from_id, const edge_t& _edge) {
            m_entries[_from_id].emplace_back(_edge);
        }

        /**
         * Removes a specified edge associated with a given vertex identifier
         * from the graph. If the last edge of the vertex is removed, the vertex
         * entry itself will be erased.
         *
         * @param [in] _from_id The identifier of the source node. The function
         *                 checks whether this vertex exists in the graph
         *                 before attempting to remove the edge.
         * @param [in] _edge The edge to be removed. It is matched against
         *                 existing edges connected to the specified vertex.
         */
        [[maybe_unused]] constexpr void remove(index_t _from_id, const edge_t& _edge) noexcept {

            if (contains(_from_id)) {

                auto& bucket = m_entries[_from_id];
                bucket.erase(std::remove(bucket.begin(), bucket.end(), _edge), bucket.end());

                if (bucket.empty()) {
                    m_entries.erase(_from_id);
                }
            }
        }

        /**
         * @brief Prunes the graph of transitory nodes.
         * @details Removes nodes from the graph that have exactly two neighbours, merging their
         *          connections with adjacent nodes to simplify the structure of the graph. This
         *          operation is repeated until no further nodes can be pruned.\n\n
         *          The algorithm adjusts the connections between neighbouring nodes before
         *          removing nodes with degree 2. This includes updating edge weights by summing
         *          the costs of connecting through the intermediate node.
         *
         * @remark The function directly modifies the internal graph representation.
         *         It does not support directed graphs and assumes undirected relationships
         *         between nodes. Future versions may extend support to directed graphs.
         *
         * @pre The graph must have at least three nodes for any pruning to occur.
         *
         * @post A simplified graph structure with fewer nodes will result if pruning
         *       conditions are met. Nodes with only two neighbours are eliminated, with
         *       connections appropriately merged.
         *
         * @warning This can be a slow operation. It is suggested that pruning is performed outside of
         *          hot loops or performance-critical code.
         *
         * @tparam ConsolidateAfterPrune If true, attempts to consolidate the managed heap after pruning (optional, default `true`).
         */
        template <bool ConsolidateAfterPrune = true>
        [[maybe_unused]] void prune() {

            // TODO: Extend support for pruning directed graphs.

            std::vector<index_t> nodesToRemove;

            bool dirty = false;

            do {
                nodesToRemove.clear();

                for (const auto& entry : m_entries) {

                    if (m_entries.size() > 2U) {

                        if (auto& [node, neighbours] = entry; neighbours.size() == 2U) {

                            dirty = ConsolidateAfterPrune;

                            auto it = neighbours.begin();

                            auto& [n1_id, n1_cost] = *it;
                            auto& [n2_id, n2_cost] = *(++it);

                            auto s1 = m_entries.find(n1_id);
                            auto s2 = m_entries.find(n2_id);

                            // Merge the connections from the current node with its neighbours:

                            if (s1 != m_entries.end() && n1_id != n2_id) {
                                auto& set = s1->second;
                                auto  sn  = std::find(set.begin(), set.end(), std::make_pair(node, n1_cost));

                                if (sn != set.end()) {

                                    if (neighbour_t new_vertex { n2_id, sn->second + n2_cost };
                                        std::find(set.begin(), set.end(), new_vertex) == set.end()
                                    ) {
                                        set.emplace_back(new_vertex);
                                    }
                                }
                            }

                            if (s2 != m_entries.end() && n1_id != n2_id) {
                                auto& set = s2->second;
                                auto  sn  = std::find(set.begin(), set.end(), std::make_pair(node, n2_cost));

                                if (sn != set.end()) {
                                    neighbour_t new_vertex{n1_id, sn->second + n1_cost};
                                    set.erase(sn);
                                    if (std::find(set.begin(), set.end(), new_vertex) == set.end()) {
                                        set.emplace_back(new_vertex);
                                    }
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
            while (!nodesToRemove.empty() && m_entries.size() > 2U);

            if (dirty) {
                chdr::malloc_consolidate();
            }
        }

        /**
         * Retrieves the neighbours associated with a vertex in the graph.
         *
         * This method fetches a constant reference to the neighbours of the vertex with the given identifier.
         *
         * @param _id [in] The index of the node whose neighbours are to be retrieved.
         * @return A constant reference to the neighbours associated with the specified node ID.
         * @warning If the requested node does not exist within the graph, calling this function invokes undefined behaviour.
         * @see contains()
         */
        [[maybe_unused, nodiscard]] HOT constexpr const neighbours_t& get_neighbours(index_t _id) const {
            assert(contains(_id) && "Node with the specified ID does not exist in the graph.");
            return m_entries.find(_id)->second;
        }

        /**
         * Checks if the specified vertex exists within the graph.
         *
         * @param [in] _id Identifier of the vertex to search for.
         * @return `true` if the vertex exists, `false` otherwise.
         */
        [[maybe_unused, nodiscard]] HOT constexpr bool contains(index_t _id) const noexcept {
            return m_entries.find(_id) != m_entries.end();
        }

        /**
         * @brief Returns the total number of vertices in the graph.
         *
         * @return The number of vertices within the graph.
         */
        [[maybe_unused, nodiscard]] constexpr size_t count() const noexcept {
            return m_entries.size();
        }

        /**
         * @brief Clears all entries from the graph.
         *
         * @details Removes all elements from the internal entries container, leaving the graph empty.
         */
        [[maybe_unused]] void clear() noexcept {
            m_entries.clear();
        }

        /**
         * @brief Retrieves the vertex at a specified index.
         * @param _id Index of the vertex to retrieve.
         * @warning If the specified index is out of bounds, calling this function is undefined behaviour.
         * @return Vertex at a specified index within the grid.
         * @see contains()
         * @see at()
         */
        [[nodiscard]] HOT constexpr auto& operator[](size_t _id) const noexcept {
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