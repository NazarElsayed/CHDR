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

#include "../types/coord.hpp"
#include "../utils/utils.hpp"
#include "base/imaze.hpp"
#include "nodes/weighted_node.hpp"
#include "utils/intrinsics.hpp"

namespace chdr::mazes {

    /**
     * @tparam Kd Dimensionality of the grid.
     */
    template<size_t Kd, typename weight_t = bool>
    class grid final : public imaze<weighted_node<weight_t>, size_t> {

        using coord_t = coord<size_t, Kd>;

    public:

        static constexpr auto s_rank { Kd };

        static_assert(              Kd > 0U, "Kd must be greater than 0.");
        static_assert(std::is_integral_v<weight_t>, "T must be integral."       );

    private:

        static constexpr auto s_neighbour_count { utils::powui(static_cast<size_t>(3U), Kd) - 1U };

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

        [[nodiscard]] constexpr size_t count() const noexcept override { return m_count; }

        template<bool IncludeDiagonals = false, typename... Args>
        [[nodiscard]] constexpr auto get_neighbours(const Args&... _id) const noexcept {
            return get_neighbours<IncludeDiagonals>({ _id... });
        }

        template<bool IncludeDiagonals = false>
        [[nodiscard]] constexpr auto get_neighbours(const coord_t& _id) const noexcept {

            if constexpr (IncludeDiagonals) {
                return compute_diagonal_neighbours(_id, std::make_index_sequence<s_neighbour_count>{});
            }
            else {
                return compute_axis_neighbours(_id, std::make_index_sequence<Kd>{});
            }
        }

        template<bool IncludeDiagonals = false>
        [[nodiscard]] constexpr auto get_neighbours(const size_t& _id) const noexcept {
            return get_neighbours<IncludeDiagonals>(utils::to_nd<size_t, Kd>(_id, size()));
        }

        template<typename... Args>
        [[nodiscard]] constexpr const auto& at(const Args&... _id) const {
            return at({ _id... });
        }

        [[nodiscard]] constexpr const auto& at(const coord_t& _id) const {
            return at(utils::to_1d(_id, m_size));
        }

        [[nodiscard]] constexpr const auto& at(const size_t& _id) const {
            assert(contains(_id) && "Out of bounds access.");
            return reinterpret_cast<const weighted_node<weight_t>&>(m_nodes[_id]);
        }

        template<typename... Args>
        [[nodiscard]] constexpr bool contains(const Args&... _id) const noexcept {
            return contains({ _id... });
        }

        [[nodiscard]] constexpr bool contains(const coord_t& _id) const noexcept {
            return std::all_of(0U, Kd, [&](const size_t& _i) noexcept { return _id[_i] < m_size[_i]; });
        }

        [[nodiscard]] constexpr bool contains(const size_t& _id) const noexcept override {
            return _id < count();
        }

        [[nodiscard]] constexpr bool is_transitory(const size_t& _index) const noexcept {

            size_t count = 0U;

            for (const auto& [nActive, nCoord] : get_neighbours(_index)) {
                if (nActive && ++count > 2U) {
                    break;
                }
            }

            return count == 2U;
        }

        [[nodiscard]] constexpr auto& operator[](const size_t& _id) const noexcept {
            return at(_id);
        }

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

            std::array<std::pair<bool, coord_t>, s_neighbour_count> result;
            (compute_single_diagonal<Indices>(_id, result[Indices]), ...);

            return result;
        }

        template<size_t... Indices>
        [[nodiscard]] constexpr auto compute_axis_neighbours(const coord_t& _id, std::index_sequence<Indices...>) const noexcept { // NOLINT(*-named-parameter)

            std::array<std::pair<bool, coord_t>, Kd * 2U> result;
            (compute_single_axis<Indices>(_id, result[Indices], result[Kd + Indices]), ...);

            return result;
        }

        template<size_t Index>
        constexpr void compute_single_diagonal(const coord_t& _id, std::pair<bool, coord_t>& _output) const noexcept {

            constexpr  size_t sampleIndex = (Index >= s_neighbour_count / 2U) ? (Index + 1U) : Index;
            constexpr coord_t direction   = utils::to_nd<size_t, Kd>(sampleIndex, { 3U });

            bool    oob    = false;
            coord_t nCoord = _id;

            IVDEP
            for (size_t j = 0U; j < Kd; ++j) {

                nCoord[j] += (direction[j] - 1U);
                      oob |= (nCoord[j] >= m_size[j]);

                if constexpr (Kd > 4U) {
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

} //chdr::mazes

#endif //CHDR_GRID_HPP