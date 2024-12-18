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
#include <cmath>
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
    template<size_t Kd, typename T = uint32_t>
    class grid final : public imaze<weighted_node<T>, size_t> {

        using coord_t = coord<size_t, Kd>;

    public:

        static constexpr auto s_rank = Kd;

        static_assert(              Kd > 0U, "Kd must be greater than 0.");
        static_assert(std::is_integral_v<T>, "T must be integral."       );

    private:

        coord_t m_size;

        std::vector<weighted_node<T>> m_nodes;

    public:

        constexpr grid(const coord_t& _size) :
            m_size(_size),
            m_nodes(count()) {}

        constexpr grid(const coord_t& _size, const std::vector<weighted_node<T>>& _nodes) :
            m_size(_size),
            m_nodes(_nodes) {}

        template <typename... Args>
        constexpr grid(const Args&... _size) :
            m_size {_size... },
            m_nodes(count())
        {
            static_assert(sizeof...(Args) == s_rank, "Number of arguments must equal the Grid's rank.");
        }

        template <typename... Args>
        constexpr grid(const Args&... _size, const std::vector<weighted_node<T>>& _nodes) :
            m_size {_size... },
            m_nodes(_nodes)
        {
            static_assert(sizeof...(Args) == s_rank, "Number of arguments must equal the Grid's rank.");
        }

        [[nodiscard]]
        constexpr const std::vector<weighted_node<T>>& nodes() const noexcept {
            return m_nodes;
        }

        constexpr void nodes(const std::vector<weighted_node<T>>& _value) noexcept {
            m_nodes = _value;
        }

        [[nodiscard]]
        constexpr const coord_t& size() const noexcept {
            return m_size;
        }

        [[nodiscard]]
        constexpr size_t count() const noexcept override {
            return utils::product<size_t>(m_size);
        }

        template<bool IncludeDiagonals = false, typename... Args>
        [[nodiscard]]
        constexpr auto get_neighbours(const Args&... _id) const noexcept {
            return get_neighbours<IncludeDiagonals>({ _id... });
        }

        template<bool IncludeDiagonals = false>
        [[nodiscard]]
        constexpr auto get_neighbours(const coord_t& _id) const noexcept {

            constexpr size_t neighbours = IncludeDiagonals ?
                static_cast<size_t>(std::pow(3U, Kd)) - 1U : Kd * 2U;

            auto result = std::array<std::pair<bool, coord_t>, neighbours>();

            if constexpr (IncludeDiagonals) {

                constexpr coord_t kernelSize = utils::build_array<size_t, Kd, 3U>();

                for (size_t i = 0U; i < neighbours; i++) {

                    const size_t sampleIndex = i < neighbours / 2U ? i : i + 1U;

                    coord_t direction = utils::to_nd<size_t, Kd>(sampleIndex, kernelSize);

                    bool oob = false;

                    coord_t nCoord;

                    IVDEP
                    VECTOR_ALWAYS
                    for (size_t j = 0U; j < Kd; j++) {

                        nCoord[j] = _id[j] + (direction[j] - 1U);

                        if (nCoord[j] < 0U || nCoord[j] > m_size[j]) {
                            oob = true;
                            break;
                        }
                    }

                    result[i].first = !oob && at(nCoord).is_active();
                    result[i].second = nCoord;
                }
            }
            else {

                IVDEP
                VECTOR_ALWAYS
                for (size_t i = 0U; i < Kd; ++i) {

                    coord_t nCoord = _id;
                    coord_t pCoord = _id;

                    --nCoord[i];
                    ++pCoord[i];

                    result[i].first  = _id[i] > 0U && at(nCoord).is_active();
                    result[i].second = nCoord;

                    result[Kd + i].first  = _id[i] < m_size[i] - 1U && at(pCoord).is_active();
                    result[Kd + i].second = pCoord;
                }
            }

            return result;
        }

        template<bool IncludeDiagonals = false>
        [[nodiscard]]
        constexpr auto get_neighbours(const size_t& _id) const noexcept {
            return get_neighbours<IncludeDiagonals>(utils::to_nd<size_t, Kd>(_id, size()));
        }

        template<typename... Args>
        [[nodiscard]]
        constexpr const weighted_node<T>& at(const Args&... _id) const {
            return at({ _id... });
        }

        [[nodiscard]]
        constexpr const weighted_node<T>& at(const coord_t& _id) const {

            const size_t index = utils::to_1d(_id, m_size);

#ifndef NDEBUG
            return m_nodes.at(index);
#else
            return m_nodes[index];
#endif // NDEBUG
        }

        [[nodiscard]]
        constexpr const weighted_node<T>& at(const size_t& _id) const {

#ifndef NDEBUG
            return m_nodes.at(_id);
#else
            return m_nodes[_id];
#endif // NDEBUG
        }

        template<typename... Args>
        [[nodiscard]]
        constexpr bool contains(const Args&... _id) const noexcept {
            return contains({ _id... });
        }

        [[nodiscard]]
        constexpr bool contains(const coord_t& _id) const noexcept {

            bool result = true;

            for (size_t i = 0U; i < Kd; ++i) {

                if (_id[i] >= m_size[i]) {
                    result = false;

                    break;
                }
            }

            return result;
        }

        [[nodiscard]]
        constexpr bool contains(const size_t& _id) const noexcept override {
            return _id < count();
        }

        [[nodiscard]]
        constexpr bool is_transitory(const size_t& _index) const noexcept {

            size_t count = 0U;

            for (const auto& n : get_neighbours(_index)) {
                if (const auto& [nActive, nCoord] = n; nActive && ++count > 2U) {
                    break;
                }
            }

            return count == 2U;
        }

        using               iterator_t = typename std::vector<weighted_node<T>>::iterator;
        using         const_iterator_t = typename std::vector<weighted_node<T>>::const_iterator;
        using       reverse_iterator_t = typename std::vector<weighted_node<T>>::reverse_iterator;
        using const_reverse_iterator_t = typename std::vector<weighted_node<T>>::const_reverse_iterator;

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

    };

} //chdr::mazes

#endif //CHDR_GRID_HPP