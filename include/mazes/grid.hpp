/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_GRID_HPP
#define CHDR_GRID_HPP

#include <array>
#include <cassert>
#include <cstddef>
#include <utility>
#include <vector>

#include "../utils/intrinsics.hpp"
#include "../utils/utils.hpp"
#include "nodes/weighted_node.hpp"

namespace chdr::mazes {

    template<typename coord_t, typename weight_t>
    class grid final {

    public:

        static constexpr auto s_rank = std::tuple_size_v<std::decay_t<coord_t>>;

        static_assert(std::is_integral_v<weight_t>, "weight_t must be integral.");

    private:

        static constexpr auto s_neighbour_count { utils::powui(static_cast<size_t>(3U), s_rank) - 1U };

        using neighbours_t = std::array<std::pair<bool, coord_t>, s_neighbour_count>;

        coord_t m_size;
        size_t m_count;

        std::vector<weight_t> m_nodes;

    public:

        constexpr grid(const coord_t& _size) :
            m_size(_size),
            m_count(utils::product<size_t>(m_size)),
            m_nodes(count()) {}

        constexpr grid(const coord_t& _size, const std::vector<weight_t>& _nodes) :
            m_size(_size),
            m_count(utils::product<size_t>(m_size)),
            m_nodes(_nodes) {}

        [[nodiscard]] constexpr const std::vector<weighted_node<weight_t>>& nodes() const noexcept { return m_nodes; }

        constexpr void nodes(const std::vector<weighted_node<weight_t>>& _value) noexcept {  m_nodes = _value; }

        [[nodiscard]] constexpr const coord_t& size() const noexcept { return m_size; }

        [[nodiscard]] constexpr size_t count() const noexcept { return m_count; }

        template<bool IncludeDiagonals = false>
        [[nodiscard]] constexpr auto get_neighbours(const coord_t& _id) const noexcept {

            if constexpr (IncludeDiagonals) {
                return compute_diagonal_neighbours(_id, std::make_index_sequence<s_neighbour_count>{});
            }
            else {
                return compute_axis_neighbours(_id, std::make_index_sequence<s_rank>{});
            }
        }

        template<bool IncludeDiagonals = false>
        [[nodiscard]] constexpr auto get_neighbours(const size_t& _id) const noexcept {
            return get_neighbours<IncludeDiagonals>(utils::to_nd(_id, size()));
        }

        [[nodiscard]] constexpr const auto& at(const coord_t& _id) const { return at(utils::to_1d(_id, m_size)); }

        [[nodiscard]] constexpr const auto& at(const size_t& _id) const {
            assert(contains(_id) && "Out of bounds access.");
            return reinterpret_cast<const weighted_node<weight_t>&>(m_nodes[_id]);
        }

        [[nodiscard]] constexpr bool contains(const coord_t& _id) const noexcept {
            return std::all_of(0U, s_rank, [&](const size_t& _i) noexcept { return _id[_i] < m_size[_i]; });
        }

        [[nodiscard]] constexpr bool contains(const size_t& _id) const noexcept { return _id < count(); }

        [[nodiscard]] constexpr bool is_transitory(const size_t& _index) const noexcept {

            size_t count = 0U;

            for (const auto& [nActive, nCoord] : get_neighbours(_index)) {
                if (nActive && ++count > 2U) {
                    break;
                }
            }

            return count == 2U;
        }

        [[nodiscard]] constexpr bool is_transitory(const neighbours_t& _neighbours) const noexcept {

            size_t count = 0U;

            for (const auto& [nActive, nCoord] : _neighbours) {
                if (nActive && ++count > 2U) {
                    break;
                }
            }

            return count == 2U;
        }

        [[nodiscard]] constexpr auto& operator[](const size_t& _id) const noexcept { return at(_id); }

        using               iterator_t = typename std::vector<weight_t>::iterator;
        using         const_iterator_t = typename std::vector<weight_t>::const_iterator;
        using       reverse_iterator_t = typename std::vector<weight_t>::reverse_iterator;
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

            neighbours_t result;
            (compute_single_diagonal<Indices>(_id, result[Indices]), ...);

            return result;
        }

        template<size_t... Indices>
        [[nodiscard]] constexpr auto compute_axis_neighbours(const coord_t& _id, std::index_sequence<Indices...>) const noexcept { // NOLINT(*-named-parameter)

            neighbours_t result;
            (compute_single_axis<Indices>(_id, result[Indices], result[s_rank + Indices]), ...);

            return result;
        }

        template<size_t Index>
        constexpr void compute_single_diagonal(const coord_t& _id, std::pair<bool, coord_t>& _output) const noexcept {

            constexpr  size_t sampleIndex = (Index >= s_neighbour_count / 2U) ? (Index + 1U) : Index;
            constexpr coord_t direction   = utils::to_nd(sampleIndex, coord_t { 3U });

            bool    oob    = false;
            coord_t nCoord = _id;

            IVDEP
            for (size_t j = 0U; j < s_rank; ++j) {

                nCoord[j] += (direction[j] - 1U);
                      oob |= (nCoord[j] >= m_size[j]);

                if constexpr (s_rank > 4U) {
                    if (oob) { break; }
                }
            }

            _output = { !oob && at(nCoord).is_active(), nCoord };
        }

        template<size_t Index>
        constexpr void compute_single_axis(const coord_t& _id, std::pair<bool, coord_t>& _negative, std::pair<bool, coord_t>& _positive) const noexcept {

            coord_t nCoord = _id; // Negative
            coord_t pCoord = _id; // Positive

            --nCoord[Index];
            ++pCoord[Index];

            _negative = { _id[Index] > 0U                 && at(utils::to_1d(nCoord, m_size)).is_active(), nCoord };
            _positive = { _id[Index] < m_size[Index] - 1U && at(utils::to_1d(pCoord, m_size)).is_active(), pCoord };
        }
    };

    /**
     * Specialization of grid for weight_t = bool
     * @tparam s_rank Dimensionality of the grid.
     */
    template <typename coord_t>
    class grid<coord_t, bool> final {

    public:

        static constexpr auto s_rank = std::tuple_size_v<std::decay_t<coord_t>>;

    private:

        static constexpr auto s_neighbour_count{utils::powui(static_cast<size_t>(3U), s_rank) - 1U};

        using neighbours_t = std::array<std::pair<bool, coord_t>, s_neighbour_count>;

        coord_t m_size;
        size_t  m_count;

        std::vector<bool> m_nodes;

    public:

        constexpr grid(const coord_t& _size) :
            m_size(_size),
            m_count(utils::product<size_t>(m_size)),
            m_nodes(count()) {}

        constexpr grid(const coord_t& _size, const std::vector<bool>& _nodes) :
            m_size(_size),
            m_count(utils::product<size_t>(m_size)),
            m_nodes(_nodes) {}

        [[nodiscard]] constexpr const std::vector<bool>& nodes() const noexcept { return m_nodes; }

        constexpr void nodes(const std::vector<bool>& _value) noexcept { m_nodes = _value; }

        [[nodiscard]] constexpr const coord_t& size() const noexcept { return m_size; }

        [[nodiscard]] constexpr size_t count() const noexcept { return m_count; }

        template <bool IncludeDiagonals = false>
        [[nodiscard]] constexpr auto get_neighbours(const coord_t& _id) const noexcept {

            if constexpr (IncludeDiagonals) {
                return compute_diagonal_neighbours(_id, std::make_index_sequence<s_neighbour_count>{});
            }
            else {
                return compute_axis_neighbours(_id, std::make_index_sequence<s_rank>{});
            }
        }

        template <bool IncludeDiagonals = false>
        [[nodiscard]] constexpr auto get_neighbours(const size_t& _id) const noexcept {
            return get_neighbours<IncludeDiagonals>(utils::to_nd(_id, size()));
        }

        [[nodiscard]] constexpr auto at(const coord_t& _id) const { return at(utils::to_1d(_id, m_size)); }

        [[nodiscard]] constexpr weighted_node<bool> at(const size_t& _id) const {
            assert(contains(_id) && "Out of bounds access.");
            return { m_nodes[_id] };
        }

        [[nodiscard]] constexpr bool contains(const coord_t& _id) const noexcept {
            return std::all_of(0U, s_rank, [&](const size_t& _i) noexcept { return _id[_i] < m_size[_i]; });
        }

        [[nodiscard]] constexpr bool contains(const size_t& _id) const noexcept { return _id < count(); }

        [[nodiscard]] constexpr bool is_transitory(const size_t& _index) const noexcept {
            size_t count = 0U;

            for (const auto& [nActive, nCoord] : get_neighbours(_index)) {
                if (nActive && ++count > 2U) {
                    break;
                }
            }

            return count == 2U;
        }

        [[nodiscard]] constexpr bool is_transitory(const neighbours_t& _neighbours) const noexcept {
            size_t count = 0U;

            for (const auto& [nActive, nCoord] : _neighbours) {
                if (nActive && ++count > 2U) {
                    break;
                }
            }

            return count == 2U;
        }

        [[nodiscard]] constexpr auto& operator[](const size_t& _id) const noexcept { return m_nodes[_id]; }

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
        [[nodiscard]] constexpr auto compute_diagonal_neighbours(const coord_t& _id, std::index_sequence<Indices...>) const noexcept {
            neighbours_t result;
            (compute_single_diagonal<Indices>(_id, result[Indices]), ...);
            return result;
        }

        template <size_t... Indices>
        [[nodiscard]] constexpr auto compute_axis_neighbours(const coord_t& _id, std::index_sequence<Indices...>) const noexcept {
            neighbours_t result;
            (compute_single_axis<Indices>(_id, result[Indices], result[s_rank + Indices]), ...);
            return result;
        }

        template <size_t Index>
        constexpr void compute_single_diagonal(const coord_t& _id, std::pair<bool, coord_t>& _output) const noexcept {

            constexpr size_t  sampleIndex = (Index >= s_neighbour_count / 2U) ? (Index + 1U) : Index;
            constexpr coord_t direction   = utils::to_nd(sampleIndex, coord_t { 3U });

            bool    oob    = false;
            coord_t nCoord = _id;

            for (size_t j = 0U; j < s_rank; ++j) {
                nCoord[j] += (direction[j] - 1U);
                oob |= (nCoord[j] >= m_size[j]);

                if constexpr (s_rank > 4U) {
                    if (oob) { break; }
                }
            }

            _output = { !oob && at(nCoord).is_active(), nCoord };
        }

        template <size_t Index>
        constexpr void compute_single_axis(const coord_t& _id, std::pair<bool, coord_t>& _negative, std::pair<bool, coord_t>& _positive) const noexcept {

            coord_t nCoord = _id; // Negative
            coord_t pCoord = _id; // Positive

            --nCoord[Index];
            ++pCoord[Index];

            _negative = { _id[Index] > 0U                 && at(utils::to_1d(nCoord, m_size)).is_active(), nCoord };
            _positive = { _id[Index] < m_size[Index] - 1U && at(utils::to_1d(pCoord, m_size)).is_active(), pCoord };
        }
    };
    
} //chdr::mazes

#endif //CHDR_GRID_HPP