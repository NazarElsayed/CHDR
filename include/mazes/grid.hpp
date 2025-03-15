/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_GRID_HPP
#define CHDR_GRID_HPP

/**
 * @file grid.hpp
 */

#include <algorithm>
#include <array>
#include <cassert>
#include <cstddef>
#include <type_traits>
#include <utility>
#include <vector>

#include "../utils/utils.hpp"
#include "nodes/weighted_node.hpp"

// ReSharper disable once CppUnusedIncludeDirective
#include "../utils/intrinsics.hpp" // NOLINT(*-include-cleaner)
#include "include/types/containers/coord.hpp"

namespace chdr::mazes {

    /**
     * @nosubgrouping
     * @class grid
     * @brief Represents a uniform-cost, bidirectional grid in K-dimensions.
     *
     * @note This class uses a `coord_t` type for grid dimensions and indexing, and a `weight_t` type to
     *       represent the weight or state of the nodes in the grid.
     *
     * @tparam coord_t Type representing coordinates.
     * @tparam weight_t Type representing node weights.
     */
    template<typename coord_t, typename weight_t>
    class grid final {

        static_assert(std::is_integral_v<weight_t>, "weight_t must be integral.");

    public:

        /**
         * @brief Represents the number of dimensions (rank) of the grid.
         *
         * @details Determines the rank (or dimensionality) of the grid by evaluating the size of the
         *          `coord_t` type.
         *          For example, a 2D grid has a rank of 2, while a 3D grid has a rank of 3.
         *
         * @remarks This value is `constexpr`.
         */
        static constexpr auto s_rank = std::tuple_size_v<std::decay_t<coord_t>>;

    private:

        /**
         * @brief Represents the total number of neighbouring cells in a multidimensional grid.
         *
         * @details Represents the number of neighbouring cells for a given grid rank (`s_rank`), excluding the cell itself.
         *          It is calculated using the formula `3^s_rank - 1`, where `s_rank` denotes the dimensionality of the grid.
         *          For example, in two dimensions (2D), the value represents 8 neighbouring cells, while in three dimensions (3D), it
         *          represents 26 neighbouring cells.
         *
         * @remarks This value is `constexpr`.
         */
        static constexpr auto s_neighbour_count { utils::powui(static_cast<size_t>(3U), s_rank) - 1U };

        coord_t m_size;
        size_t m_count;

        std::vector<weight_t> m_nodes;

    public:

        using  neighbour_t = std::pair<bool, coord_t>;
        using neighbours_t = std::array<neighbour_t, s_neighbour_count>;

        /**
         * @name Constructors
         * @{
         */

        /**
         * @brief Constructs a grid instance with a specified size.
         *
         * @details Initialises the grid with the given dimensions.
         *
         * @param [in] _size The dimensions of the grid, represented as a `coord_t` type.
         */
        constexpr grid(const coord_t& _size) :
            m_size(_size),
            m_count(utils::product<size_t>(m_size)),
            m_nodes(count()) {}

        /**
         * @brief Constructs a grid instance with the specified size and initial node values.
         *
         * @details Initialises a `grid` object with its dimensions `_size` and the
         *          corresponding node values `_nodes`.
         *          
         * @note The number of nodes provided must match the total number of cells in the grid.
         *
         * @param [in] _size The dimensions of the grid, represented as a `coord_t` type.
         * @param [in] _nodes A vector of node values.
         *
         * @warning The number of nodes provided must match the number of cells in the grid.
         */
        constexpr grid(const coord_t& _size, const std::vector<weight_t>& _nodes) :
            m_size(_size),
            m_count(utils::product<size_t>(m_size)),
            m_nodes(_nodes)
        {
            assert(m_nodes.size() == m_count && "The number of nodes must match the number of cells in the grid.");
        }

        /**
         * @brief Constructs a grid instance with specified size and initial node values using move semantics.
         *
         * @details Initialises a `grid` object with its dimensions `_size` and the
         *          corresponding node values `_nodes`, which are emplaced using move semantics.
         *
         * @param[in] _size The dimensions of the grid, represented as a `coord_t` type.
         * @param[in] _nodes An rvalue reference to a vector of node values.
         *
         * @warning The number of nodes provided must match the number of cells in the grid.
         */
        constexpr grid(const coord_t& _size, std::vector<weight_t>&& _nodes) :
            m_size(_size),
            m_count(utils::product<size_t>(m_size)),
            m_nodes(std::move(_nodes))
        {
            assert(m_nodes.size() == m_count && "The number of nodes must match the number of cells in the grid.");
        }

        ~grid() = default;

        constexpr grid           (const grid&) = delete;
        constexpr grid& operator=(const grid&) = delete;

        [[nodiscard]] constexpr grid(grid&&) noexcept = default;

#if __cplusplus > 202302L
        constexpr
#endif
        grid& operator=(grid&&) noexcept = default;

        /**
         * @}
         */

        /**
         * @brief Retrieves a constant reference to the vector of nodes in the grid.
         *
         * @details Provides access to all nodes within the grid. Each node is represented by a `weight_t` object,
         *          which contains the associated weight and other relevant properties. The returned vector maintains
         *          all nodes in internal storage, preserving their current state.
         *
         * @return A vector of objects representing the nodes in the grid.
         */
        [[nodiscard]] constexpr const std::vector<weight_t>& nodes() const noexcept { return m_nodes; }

        /**
         * @brief Sets the nodes for the grid.
         *
         * @details Assigns a new collection of nodes to the grid.
         *
         * @param _value [in] A vector containing the new values to be assigned to the grid nodes.
         * @warning The number of nodes provided must match the number of cells in the grid.
         */
        constexpr void nodes(const std::vector<weight_t>& _value) noexcept {
            assert(m_nodes.size() == m_count && "The number of nodes must match the number of cells in the grid.");
            m_nodes = _value;
        }

        /**
         * @brief Retrieves the size of the grid.
         *
         * @details Provides access to the size of the grid.
         *          The returned value reflects the dimensional bounds of the grid.
         *
         * @return The size of the grid.
         */
        [[nodiscard]] constexpr const coord_t& size() const noexcept { return m_size; }

        /**
         * @brief Retrieves the total count of elements in the grid.
         * @return The number of elements in the grid.
         */
        [[nodiscard]] constexpr size_t count() const noexcept { return m_count; }

        /**
         * @brief Retrieves the neighbours of a node within the grid.
         *
         * @details Computes the neighbours of the specified node in the grid, either including or
         *          excluding diagonal neighbours. The neighbours allow navigation and provide information
         *          about the connectivity of the node to its surroundings.
         *
         * @param _id The coordinate of the node for which the neighbours are to be retrieved.
         *
         * @tparam IncludeDiagonals Boolean template parameter indicating whether to include diagonal neighbours.
         *         - If `true`, diagonal neighbours are included.
         *         - If `false`, only axis-aligned neighbours are considered.
         *
         * @note Out-of-bounds neighbours are marked inactive.
         *       Activity of a neighbour is determined by its state within the grid.
         *       The total number of neighbours depends on the dimensionality (rank) of the grid and 
         *       whether diagonal neighbours are included.
         *
         * @return An array of pairs where each pair represents:
         *         - A boolean indicating whether the neighbour is "active".
         *         - The coordinate of the neighbour (invalid if the node is inactive).
         *
         * @warning In the resulting array of pairs, if a node is inactive, its corresponding coordinate is invalid and should be used.
         */
        template<bool IncludeDiagonals = false>
        [[nodiscard]] HOT constexpr auto get_neighbours(const coord_t& _id) const noexcept {

            if constexpr (IncludeDiagonals) {
                return compute_diagonal_neighbours(_id, std::make_index_sequence<s_neighbour_count>{});
            }
            else {
                return compute_axis_neighbours(_id, std::make_index_sequence<s_rank>{});
            }
        }

        /**
         * @brief Retrieves the neighbours of a node within the grid.
         *
         * @details Computes the neighbours of the specified node in the grid, either including or
         *          excluding diagonal neighbours. The neighbours allow navigation and provide information
         *          about the connectivity of the node to its surroundings.
         *
         * @param _id Index of the node for which the neighbours are to be retrieved.
         *
         * @tparam IncludeDiagonals Boolean template parameter indicating whether to include diagonal neighbours.
         *         - If `true`, diagonal neighbours are included.
         *         - If `false`, only axis-aligned neighbours are considered.
         *
         * @note Out-of-bounds neighbours are marked inactive.
         *       Activity of a neighbour is determined by its state within the grid.
         *       The total number of neighbours depends on the dimensionality (rank) of the grid and
         *       whether diagonal neighbours are included.
         *
         * @return An array of pairs where each pair represents:
         *         - A boolean indicating whether the neighbour is "active".
         *         - The coordinate of the neighbour (invalid if the node is inactive).
         *
         * @warning In the resulting array of pairs, if a node is inactive, its corresponding coordinate is invalid and should be used.
         */
        template<bool IncludeDiagonals = false>
        [[nodiscard]] HOT constexpr auto get_neighbours(size_t _id) const noexcept {
            return get_neighbours<IncludeDiagonals>(utils::to_nd(_id, size()));
        }

        constexpr auto check_neighbour(const coord_t& _id, const coord_t& _direction) const noexcept {

            neighbour_t output{};

            coord_t coord = _id;
            bool oob = false;

            for (size_t i = 0U; i < s_rank; ++i) {
                coord[i] += (_direction[i] - 1U);

                if (coord[i] >= m_size[i]) {
                    oob = true;
                    break;
                }
            }

            output = { !oob && operator[](coord).is_active(), coord };

            return output;
        }

        /**
         * @brief Determines whether the provided coordinate is within the bounds of the grid.
         *
         * @details Evaluates the provided coordinate against the dimensions of the grid to ensure that all of
         *         its values fall within the grid's defined bounds. Each dimension of the coordinate is compared
         *          against the corresponding size of the grid.
         *
         * @param [in] _id The coordinate to be evaluated. It represents a multidimensional position in the grid.
         *
         * @return `true` if the provided coordinate lies within the bounds of the grid. Otherwise, `false`.
         */
        [[nodiscard]] constexpr bool contains(const coord_t& _id) const noexcept {
            return std::all_of(0U, s_rank, [&](size_t _i) noexcept { return _id[_i] < m_size[_i]; });
        }

        /**
         * @brief Determines whether the given identifier is within the bounds of the grid.
         *
         * @details Checks if the provided identifier is less than the total count of nodes,
         *          thus confirming its validity within the bounds of the grid.
         *
         * @param _id The identifier to be checked.
         * @return `true` if the identifier is within the valid range; otherwise, `false`.
         * @see count()
         */
        [[nodiscard]] constexpr bool contains(size_t _id) const noexcept { return _id < count(); }

        /**
         * @brief Determines whether a cell at the given index is transitory.
         *
         * @details Evaluates the state of a cell based on its active neighbours.
         *          A cell is considered transitory if it has precisely two active neighbours.
         *
         * @param _index The index of the cell to evaluate.
         *
         * @return True if the cell has exactly two active neighbours; otherwise, false.
         */
        [[nodiscard]] constexpr bool is_transitory(size_t _index) const noexcept {

            size_t count = 0U;

            for (const auto& [nActive, _] : get_neighbours(_index)) {

                if (nActive) {
                    if (++count > 2U) {
                        break;
                    }
                }
            }

            return count == 2U;
        }

        /**
         * @brief Determines whether a cell is transitory based on its neighbours.
         *
         * @details Evaluates the state of a cell based on its active neighbours.
         *          A cell is considered transitory if it has precisely two active neighbours.
         *
         * @param _neighbours The index of the cell to evaluate.
         *
         * @return True if the cell has exactly two active neighbours; otherwise, false.
         */
        [[nodiscard]] constexpr bool is_transitory(const neighbours_t& _neighbours) const noexcept {

            size_t count = 0U;

            for (const auto& [nActive, _] : _neighbours) {

                if (nActive) {
                    if (++count > 2U) {
                        break;
                    }
                }
            }

            return count == 2U;
        }

        /**
         * @brief Retrieves the node at a specified coordinate.
         * @param _id Coordinate of the node to retrieve.
         * @pre `_id` must reference a valid node, which exists in the grid.
         * @warning If the specified coordinate is out of bounds, calling this function is undefined behaviour.
         * @return Node at a specified coordinate within the grid.
         * @see contains()
         * @see operator[]()
         */
        [[nodiscard]] constexpr auto& operator[](const coord_t& _id) const noexcept {
            return operator[](utils::to_1d(_id, m_size));
        }

        /**
         * @brief Retrieves the node at a specified index.
         * @param _id Index of the node to retrieve.
         * @pre `_id` must reference a valid node, which exists in the grid.
         * @warning Calling this function is undefined behaviour if the specified index is out of bounds.
         * @return Node at a specified index within the grid.
         * @see contains()
         * @see operator[]()
         */
        [[nodiscard]] HOT constexpr auto& operator[](size_t _id) const noexcept {
            assert(contains(_id) && "Out of bounds access.");
            return reinterpret_cast<const weighted_node<weight_t>&>(m_nodes[_id]);
        }

        using               iterator_t = typename std::vector<weight_t>::              iterator;
        using         const_iterator_t = typename std::vector<weight_t>::        const_iterator;
        using       reverse_iterator_t = typename std::vector<weight_t>::      reverse_iterator;
        using const_reverse_iterator_t = typename std::vector<weight_t>::const_reverse_iterator;

        [[maybe_unused, nodiscard]]       iterator_t  begin()       noexcept { return m_nodes.begin();  }
        [[maybe_unused, nodiscard]] const_iterator_t  begin() const noexcept { return m_nodes.begin();  }
        [[maybe_unused, nodiscard]] const_iterator_t cbegin() const noexcept { return m_nodes.cbegin(); }

        [[maybe_unused, nodiscard]]       iterator_t  end()       noexcept { return m_nodes.end();  }
        [[maybe_unused, nodiscard]] const_iterator_t  end() const noexcept { return m_nodes.end();  }
        [[maybe_unused, nodiscard]] const_iterator_t cend() const noexcept { return m_nodes.cend(); }

        [[maybe_unused, nodiscard]]       reverse_iterator_t  rbegin()       noexcept { return m_nodes.rbegin();  }
        [[maybe_unused, nodiscard]] const_reverse_iterator_t  rbegin() const noexcept { return m_nodes.rbegin();  }
        [[maybe_unused, nodiscard]] const_reverse_iterator_t crbegin() const noexcept { return m_nodes.crbegin(); }

        [[maybe_unused, nodiscard]]       reverse_iterator_t  rend()       noexcept { return m_nodes.rend();  }
        [[maybe_unused, nodiscard]] const_reverse_iterator_t  rend() const noexcept { return m_nodes.rend();  }
        [[maybe_unused, nodiscard]] const_reverse_iterator_t crend() const noexcept { return m_nodes.crend(); }

    private:

        template<size_t... Indices>
        [[nodiscard]] constexpr auto compute_diagonal_neighbours(const coord_t& _id, std::index_sequence<Indices...>) const noexcept { // NOLINT(*-named-parameter)
            neighbours_t result{};
            (compute_single_diagonal<Indices>(_id, result[Indices]), ...);
            return result;
        }

        template<size_t... Indices>
        [[nodiscard]] constexpr auto compute_axis_neighbours(const coord_t& _id, std::index_sequence<Indices...>) const noexcept { // NOLINT(*-named-parameter)
            neighbours_t result{};
            (compute_single_axis<Indices>(_id, result[Indices], result[s_rank + Indices]), ...);
            return result;
        }

        template<size_t Index>
        constexpr void compute_single_diagonal(const coord_t& _id, neighbour_t& _output) const noexcept {

            constexpr  size_t sampleIndex = (Index >= s_neighbour_count / 2U) ? (Index + 1U) : Index; // Skips the middle value
            constexpr coord_t direction   = utils::to_nd(sampleIndex, coord_t { 3U });

            bool oob = false;
            coord_t coord = _id;

            for (size_t i = 0U; i < s_rank; ++i) {
                coord[i] += (direction[i] - 1U);

                if (coord[i] >= m_size[i]) {
                    oob = true;
                    break;
                }
            }

            _output = { !oob && operator[](coord).is_active(), coord };
        }

        template<size_t Index>
        constexpr void compute_single_axis(const coord_t& _id, neighbour_t& _negative, neighbour_t& _positive) const noexcept {

            coord_t nCoord = _id; // Negative
            coord_t pCoord = _id; // Positive

            --nCoord[Index];
            ++pCoord[Index];

            _negative = { _id[Index] > 0U                 && operator[](nCoord).is_active(), nCoord };
            _positive = { _id[Index] < m_size[Index] - 1U && operator[](pCoord).is_active(), pCoord };
        }
    };

    /**
     * @nosubgrouping
     * @brief Specialization of grid for weight_t = bool.
     * @tparam coord_t Type representing coordinates.
     * @note In this configuration, the grid is represented using a bitset.
     * @see grid
     */
    template <typename coord_t>
    class grid<coord_t, bool> final {

    public:

        /**
         * @brief Represents the number of dimensions (rank) of the grid.
         *
         * @details Determines the rank (or dimensionality) of the grid by evaluating the size of the
         *          `coord_t` type.
         *          For example, a 2D grid has a rank of 2, while a 3D grid has a rank of 3.
         *
         * @remarks This value is `constexpr`.
         */
        static constexpr auto s_rank = std::tuple_size_v<std::decay_t<coord_t>>;

    private:

        /**
         * @brief Represents the total number of neighbouring cells in a multidimensional grid.
         *
         * @details Represents the number of neighbouring cells for a given grid rank (`s_rank`), excluding the cell itself.
         *          It is calculated using the formula `3^s_rank - 1`, where `s_rank` denotes the dimensionality of the grid.
         *          For example, in two dimensions (2D), the value represents 8 neighbouring cells, while in three dimensions (3D), it
         *          represents 26 neighbouring cells.
         *
         * @remarks This value is `constexpr`.
         */
        static constexpr auto s_neighbour_count { utils::powui(static_cast<size_t>(3U), s_rank) - 1U };

        coord_t m_size;
        size_t  m_count;

        std::vector<bool> m_nodes;

    public:

        using  neighbour_t = std::pair<bool,  coord_t>;
        using neighbours_t = std::array<neighbour_t, s_neighbour_count>;

        /**
         * @name Constructors
         * @{
         */

        /**
         * @brief Constructs a grid instance with a specified size.
         *
         * @details Initialises the grid with the given dimensions.
         *
         * @param[in] _size The dimensions of the grid, represented as a `coord_t` type.
         */
        constexpr grid(const coord_t& _size) :
            m_size(_size),
            m_count(utils::product<size_t>(m_size)),
            m_nodes(count()) {}

        /**
         * @brief Constructs a grid instance with the specified size and initial node values.
         *
         * @details Initialises a `grid` object with its dimensions `_size` and the
         *          corresponding node values `_nodes`.
         *
         * @note The number of nodes provided must match the total number of cells in the grid.
         *
         * @param[in] _size The dimensions of the grid, represented as a `coord_t` type.
         * @param[in] _nodes A vector of node values.
         *
         * @warning The number of nodes provided must match the number of cells in the grid.
         */
        constexpr grid(const coord_t& _size, const std::vector<bool>& _nodes) :
            m_size(_size),
            m_count(utils::product<size_t>(m_size)),
            m_nodes(_nodes) {}

        /**
         * @brief Constructs a grid instance with specified size and initial node values using move semantics.
         *
         * @details Initialises a `grid` object with its dimensions `_size` and the
         *          corresponding node values `_nodes`, which are emplaced using move semantics.
         *
         * @param[in] _size The dimensions of the grid, represented as a `coord_t` type.
         * @param[in] _nodes A rvalue reference to a vector of node values.
         *
         * @warning The number of nodes provided must match the number of cells in the grid.
         */
        constexpr grid(const coord_t& _size, const std::vector<bool>&& _nodes) :
            m_size(_size),
            m_count(utils::product<size_t>(m_size)),
            m_nodes(_nodes) {}

        ~grid() = default;

        constexpr grid           (const grid&) = delete;
        constexpr grid& operator=(const grid&) = delete;

        [[nodiscard]] constexpr grid(grid&&) noexcept = default;

#if __cplusplus > 202302L
        constexpr
#endif
        grid& operator=(grid&&) noexcept = default;

        /**
         * @}
         */

        /**
         * @brief Retrieves a constant reference to the vector of nodes in the grid.
         *
         * @details Provides access to all nodes within the grid. Each node is represented by a `weight_t` object,
         *          which contains the associated weight and other relevant properties. The returned vector maintains
         *          all nodes in internal storage, preserving their current state.
         *
         * @return A vector of objects representing the nodes in the grid.
         */
        [[nodiscard]] constexpr const std::vector<bool>& nodes() const noexcept { return m_nodes; }

        /**
         * @brief Sets the nodes for the grid.
         *
         * @details Assigns a new collection of nodes to the grid.
         *
         * @param _value [in] A vector containing the new values to be assigned to the grid nodes.
         * @warning The number of nodes provided must match the number of cells in the grid.
         */
        constexpr void nodes(const std::vector<bool>& _value) noexcept { m_nodes = _value; }

        /**
         * @brief Retrieves the size of the grid.
         *
         * @details Provides access to the size of the grid.
         *          The returned value reflects the dimensional bounds of the grid.
         *
         * @return The size of the grid.
         */
        [[nodiscard]] constexpr const coord_t& size() const noexcept { return m_size; }

        /**
         * @brief Retrieves the total count of elements in the grid.
         * @return The number of elements in the grid.
         */
        [[nodiscard]] constexpr size_t count() const noexcept { return m_count; }

        /**
         * @brief Retrieves the neighbours of a node within the grid.
         *
         * @details Computes the neighbours of the specified node in the grid, either including or
         *          excluding diagonal neighbours. The neighbours allow navigation and provide information
         *          about the connectivity of the node to its surroundings.
         *
         * @param _id The coordinate of the node for which the neighbours are to be retrieved.
         *
         * @tparam IncludeDiagonals Boolean template parameter indicating whether to include diagonal neighbours.
         *         - If `true`, diagonal neighbours are included.
         *         - If `false`, only axis-aligned neighbours are considered.
         *
         * @note Out-of-bounds neighbours are marked inactive.
         *       Activity of a neighbour is determined by its state within the grid.
         *       The total number of neighbours depends on the dimensionality (rank) of the grid and
         *       whether diagonal neighbours are included.
         *
         * @return An array of pairs where each pair represents:
         *         - A boolean indicating whether the neighbour is "active".
         *         - The coordinate of the neighbour (invalid if the node is inactive).
         *
         * @warning In the resulting array of pairs, if a node is inactive, its corresponding coordinate is invalid and should be used.
         */
        template <bool IncludeDiagonals = false>
        [[nodiscard]] HOT constexpr auto get_neighbours(const coord_t& _id) const noexcept {

            if constexpr (IncludeDiagonals) {
                return compute_diagonal_neighbours(_id, std::make_index_sequence<s_neighbour_count>{});
            }
            else {
                return compute_axis_neighbours(_id, std::make_index_sequence<s_rank>{});
            }
        }

        /**
         * @brief Retrieves the neighbours of a node within the grid.
         *
         * @details Computes the neighbours of the specified node in the grid, either including or
         *          excluding diagonal neighbours. The neighbours allow navigation and provide information
         *          about the connectivity of the node to its surroundings.
         *
         * @param _id Index of the node for which the neighbours are to be retrieved.
         *
         * @tparam IncludeDiagonals Boolean template parameter indicating whether to include diagonal neighbours.
         *         - If `true`, diagonal neighbours are included.
         *         - If `false`, only axis-aligned neighbours are considered.
         *
         * @note Out-of-bounds neighbours are marked inactive.
         *       Activity of a neighbour is determined by its state within the grid.
         *       The total number of neighbours depends on the dimensionality (rank) of the grid and
         *       whether diagonal neighbours are included.
         *
         * @return An array of pairs where each pair represents:
         *         - A boolean indicating whether the neighbour is "active".
         *         - The coordinate of the neighbour (invalid if the node is inactive).
         *
         * @warning In the resulting array of pairs, if a node is inactive, its corresponding coordinate is invalid and should be used.
         */
        template <bool IncludeDiagonals = false>
        [[nodiscard]] HOT constexpr auto get_neighbours(size_t _id) const noexcept {
            return get_neighbours<IncludeDiagonals>(utils::to_nd(_id, size()));
        }

        constexpr auto check_neighbour(const coord_t& _id, const coord_t& _direction) const noexcept {

            neighbour_t output{};

            coord_t coord = _id;
            bool oob = false;

            for (size_t i = 0U; i < s_rank; ++i) {
                coord[i] += (_direction[i] - 1U);

                if (coord[i] >= m_size[i]) {
                    oob = true;
                    break;
                }
            }

            output = { !oob && operator[](coord).is_active(), coord };

            return output;
        }

        /**
         * @brief Determines whether the provided coordinate is within the bounds of the grid.
         *
         * @details Evaluates the provided coordinate against the dimensions of the grid to ensure that all of
         *         its values fall within the grid's defined bounds. Each dimension of the coordinate is compared
         *          against the corresponding size of the grid.
         *
         * @param [in] _id The coordinate to be evaluated. It represents a multidimensional position in the grid.
         *
         * @return `true` if the provided coordinate lies within the bounds of the grid. Otherwise, `false`.
         */
        [[nodiscard]] constexpr bool contains(const coord_t& _id) const noexcept {
            return std::all_of(0U, s_rank, [&](size_t _i) noexcept { return _id[_i] < m_size[_i]; });
        }

        /**
         * @brief Determines whether the given identifier is within the bounds of the grid.
         *
         * @details Checks if the provided identifier is less than the total count of nodes,
         *          thus confirming its validity within the bounds of the grid.
         *
         * @param _id The identifier to be checked.
         * @return `true` if the identifier is within the valid range; otherwise, `false`.
         * @see count()
         */
        [[nodiscard]] constexpr bool contains(size_t _id) const noexcept { return _id < count(); }

        /**
         * @brief Determines whether a cell at the given index is transitory.
         *
         * @details Evaluates the state of a cell based on its active neighbours.
         *          A cell is considered transitory if it has precisely two active neighbours.
         *
         * @param _index The index of the cell to evaluate.
         *
         * @return True if the cell has exactly two active neighbours; otherwise, false.
         */
        [[nodiscard]] constexpr bool is_transitory(size_t _index) const noexcept {
            size_t count = 0U;

            for (const auto& [nActive, _] : get_neighbours(_index)) {

                if (nActive) {
                    if (++count > 2U) {
                        break;
                    }
                }
            }

            return count == 2U;
        }

        /**
         * @brief Determines whether a cell is transitory based on its neighbours.
         *
         * @details Evaluates the state of a cell based on its active neighbours.
         *          A cell is considered transitory if it has precisely two active neighbours.
         *
         * @param _neighbours Neighbours to evaluate.
         *
         * @return True if the cell has exactly two active neighbours; otherwise, false.
         */
        [[nodiscard]] constexpr bool is_transitory(const neighbours_t& _neighbours) const noexcept {
            size_t count = 0U;

            for (const auto& [nActive, _] : _neighbours) {

                if (nActive) {
                    if (++count > 2U) {
                        break;
                    }
                }
            }

            return count == 2U;
        }

        /**
         * @brief Retrieves the node at a specified coordinate.
         * @param [in] _id Coordinate of the node to retrieve.
         * @pre `_id` must reference a valid node, which exists in the grid.
         * @warning If the specified coordinate is out of bounds, calling this function is undefined behaviour.
         * @return Node at a specified coordinate within the grid.
         * @see contains()
         * @see operator[]()
         */
        [[nodiscard]] constexpr auto operator[](const coord_t& _id) const { return operator[](utils::to_1d(_id, m_size)); }

        /**
         * @brief Retrieves the node at a specified index.
         * @param _id Index of the node to retrieve.
         * @pre `_id` must reference a valid node, which exists in the grid.
         * @warning Calling this function is undefined behaviour if the specified index is out of bounds.
         * @return Node at a specified index within the grid.
         * @see contains()
         * @see operator[]()
         */
        [[nodiscard]] HOT constexpr auto operator[](size_t _id) const {
            assert(contains(_id) && "Out of bounds access.");
            return weighted_node { m_nodes[_id] };
        }

        using               iterator_t = std::vector<bool>::              iterator;
        using         const_iterator_t = std::vector<bool>::        const_iterator;
        using       reverse_iterator_t = std::vector<bool>::      reverse_iterator;
        using const_reverse_iterator_t = std::vector<bool>::const_reverse_iterator;

        [[maybe_unused, nodiscard]]       iterator_t  begin()       noexcept { return m_nodes.begin();  }
        [[maybe_unused, nodiscard]] const_iterator_t  begin() const noexcept { return m_nodes.begin();  }
        [[maybe_unused, nodiscard]] const_iterator_t cbegin() const noexcept { return m_nodes.cbegin(); }

        [[maybe_unused, nodiscard]]       iterator_t  end()       noexcept { return m_nodes.end();  }
        [[maybe_unused, nodiscard]] const_iterator_t  end() const noexcept { return m_nodes.end();  }
        [[maybe_unused, nodiscard]] const_iterator_t cend() const noexcept { return m_nodes.cend(); }

        [[maybe_unused, nodiscard]]       reverse_iterator_t  rbegin()       noexcept { return m_nodes.rbegin();  }
        [[maybe_unused, nodiscard]] const_reverse_iterator_t  rbegin() const noexcept { return m_nodes.rbegin();  }
        [[maybe_unused, nodiscard]] const_reverse_iterator_t crbegin() const noexcept { return m_nodes.crbegin(); }

        [[maybe_unused, nodiscard]]       reverse_iterator_t  rend()       noexcept { return m_nodes.rend();  }
        [[maybe_unused, nodiscard]] const_reverse_iterator_t  rend() const noexcept { return m_nodes.rend();  }
        [[maybe_unused, nodiscard]] const_reverse_iterator_t crend() const noexcept { return m_nodes.crend(); }

    private:

        template <size_t... Indices>
        [[nodiscard]] constexpr auto compute_diagonal_neighbours(const coord_t& _id, [[maybe_unused]] std::index_sequence<Indices...> _indices) const noexcept {
            neighbours_t result{};
            (compute_single_diagonal<Indices>(_id, result[Indices]), ...);
            return result;
        }

        template <size_t... Indices>
        [[nodiscard]] constexpr auto compute_axis_neighbours(const coord_t& _id, [[maybe_unused]] std::index_sequence<Indices...> _indices) const noexcept {
            neighbours_t result{};
            (compute_single_axis<Indices>(_id, result[Indices], result[s_rank + Indices]), ...);
            return result;
        }

        template <size_t Index>
        constexpr void compute_single_diagonal(const coord_t& _id, neighbour_t& _output) const noexcept {

            constexpr  size_t sampleIndex = (Index >= s_neighbour_count / 2U) ? (Index + 1U) : Index;
            constexpr coord_t direction   = utils::to_nd(sampleIndex, coord_t { 3U });

            bool oob = false;
            coord_t coord = _id;

            for (size_t j = 0U; j < s_rank; ++j) {
                coord[j] += (direction[j] - 1U);

                if (coord[j] >= m_size[j]) {
                    oob = true;
                    break;
                }
            }

            _output = { !oob && operator[](coord).is_active(), coord };
        }

        template <size_t Index>
        constexpr void compute_single_axis(const coord_t& _id, neighbour_t& _negative, neighbour_t& _positive) const noexcept {

            coord_t nCoord = _id; // Negative
            coord_t pCoord = _id; // Positive

            --nCoord[Index];
            ++pCoord[Index];

            _negative = { _id[Index] > 0U                 && operator[](utils::to_1d(nCoord, m_size)).is_active(), nCoord };
            _positive = { _id[Index] < m_size[Index] - 1U && operator[](utils::to_1d(pCoord, m_size)).is_active(), pCoord };
        }
    };

} //chdr::mazes

#endif //CHDR_GRID_HPP