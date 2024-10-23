/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_GRID_HPP
#define CHDR_GRID_HPP

#include "../nodes/WeightedNode.hpp"
#include "../types/Coord.hpp"
#include "../utils/Utils.hpp"
#include "base/IMaze.hpp"

#include <vector>
#include <array>
#include <cmath>

namespace CHDR::Mazes {

    /**
     * @tparam Kd Dimensionality of the grid.
     */
    template <const size_t Kd, typename T = uint32_t>
    class Grid : public IMaze<WeightedNode<T>, size_t> {

        using coord_t = Coord<size_t, Kd>;

    public:

        static constexpr auto Rank = Kd;

        static_assert(              Kd > 0U, "Kd must be greater than 0.");
        static_assert(std::is_integral_v<T>, "T must be integral."       );

    private:

        coord_t m_Size;

        std::vector<WeightedNode<T>> m_Nodes;

    public:

        constexpr Grid(const coord_t& _size) :
            m_Size(_size),
            m_Nodes(Utils::Product<size_t>(m_Size)) {}

        constexpr Grid(const coord_t& _size, const std::vector<WeightedNode<T>>& _nodes) :
            m_Size(_size),
            m_Nodes(_nodes) {}

        template <typename... Args>
        constexpr Grid(const Args&... _size) :
            m_Size { _size... },
            m_Nodes(Utils::Product<size_t>(m_Size))
        {
            static_assert(sizeof...(Args) == Rank, "Number of arguments must equal the Grid's rank.");
        }

        template <typename... Args>
        constexpr Grid(const Args&... _size, const std::vector<WeightedNode<T>>& _nodes) :
            m_Size { _size... },
            m_Nodes(_nodes)
        {
            static_assert(sizeof...(Args) == Rank, "Number of arguments must equal the Grid's rank.");
        }

        [[nodiscard]]
        constexpr const std::vector<WeightedNode<T>>& Nodes() const {
            return m_Nodes;
        }

        constexpr void Nodes(const std::vector<WeightedNode<T>>& _value) {
            m_Nodes = _value;
        }

        [[nodiscard]]
        constexpr const coord_t& Size() const {
            return m_Size;
        }

        template<typename... Args>
        constexpr void Size(const Args&... _value) {
            Size({ _value... });
        }

        constexpr void Size(const coord_t& _value) {
            m_Size = _value;
        }

        [[nodiscard]]
        constexpr size_t Count() const override {
            return Utils::Product<size_t>(m_Size);
        };

        template<bool IncludeDiagonals = false, typename... Args>
        [[nodiscard]]
        constexpr auto GetNeighbours(const Args&... _id) const {
            return GetNeighbours<IncludeDiagonals>({ _id... });
        }

        template<bool IncludeDiagonals = false>
        [[nodiscard]]
        constexpr auto GetNeighbours(const coord_t& _id) const {

            constexpr size_t neighbours = IncludeDiagonals ?
                            static_cast<size_t>(std::pow(3U, Kd)) - 1U :
                            Kd * 2U;

            auto result = std::array<std::pair<bool, coord_t>, neighbours>();

            if constexpr (IncludeDiagonals) {

                coord_t kernelSize;
                for (size_t i = 0U; i < Kd; i++) {
                    kernelSize[i] = 3U;
                }

                for (size_t i = 0U; i < neighbours; i++) {

                    size_t sampleIndex = i < neighbours / 2U ? i : i + 1U;

                    coord_t direction = Utils::ToND<size_t, Kd>(sampleIndex, kernelSize);

                    bool oob = false;

                    coord_t nCoord;
                    for (size_t j = 0U; j < Kd; j++) {

                        nCoord[j] = _id[j] + (direction[j] - 1U);

                        if (nCoord[j] < 0U || nCoord[j] > m_Size[j]) {
                            oob = true;
                            break;
                        }
                    }

                    result[i].first = !oob && At(nCoord).IsActive();
                    result[i].second = nCoord;
                }
            }
            else {

                for (size_t i = 0U; i < Kd; ++i) {

                    coord_t nCoord = _id;
                    coord_t pCoord = _id;

                    --nCoord[i];
                    ++pCoord[i];

                    result[i].first  = _id[i] > 0U && At(nCoord).IsActive();
                    result[i].second = nCoord;

                    result[Kd + i].first  = _id[i] < m_Size[i] - 1U && At(pCoord).IsActive();
                    result[Kd + i].second = pCoord;
                }
            }

            return result;
        }

        template<bool IncludeDiagonals = false>
        [[nodiscard]]
        constexpr auto GetNeighbours(const size_t& _id) const {
            return GetNeighbours<IncludeDiagonals>(Utils::ToND<size_t, Kd>(_id, Size()));
        }

        template<typename... Args>
        [[nodiscard]]
        constexpr const WeightedNode<T>& At(const Args&... _id) const {
            return At({ _id... });
        }

        [[nodiscard]]
        constexpr const WeightedNode<T>& At(const coord_t& _id) const {

            const size_t index = Utils::To1D(_id, m_Size);

#ifndef NDEBUG
            return m_Nodes.at(index);
#else
            return m_Nodes[index];
#endif // NDEBUG
        }

        [[nodiscard]]
        constexpr const WeightedNode<T>& At(const size_t& _id) const {

#ifndef NDEBUG
            return m_Nodes.at(_id);
#else
            return m_Nodes[_id];
#endif // NDEBUG
        }

        template<typename... Args>
        [[nodiscard]]
        constexpr bool Contains(const Args&... _id) const {
            return Contains({ _id... });
        }

        [[nodiscard]]
        constexpr bool Contains(const coord_t& _id) const {

            bool result = true;

            for (size_t i = 0U; i < Kd; ++i) {

                if (_id[i] >= m_Size[i]) {
                    result = false;

                    break;
                }
            }

            return result;
        }

        [[nodiscard]]
        constexpr bool Contains(const size_t& _id) const override {
            return _id < Utils::Product<size_t>(m_Size);
        }

        using               iterator = typename std::vector<WeightedNode < T>>::iterator;
        using         const_iterator = typename std::vector<WeightedNode < T>>::const_iterator;
        using       reverse_iterator = typename std::vector<WeightedNode < T>>::reverse_iterator;
        using const_reverse_iterator = typename std::vector<WeightedNode < T>>::const_reverse_iterator;

              iterator  begin()       { return m_Nodes.begin();  }
        const_iterator  begin() const { return m_Nodes.begin();  }
        const_iterator cbegin() const { return m_Nodes.cbegin(); }

              iterator  end()       { return m_Nodes.end();  }
        const_iterator  end() const { return m_Nodes.end();  }
        const_iterator cend() const { return m_Nodes.cend(); }

              reverse_iterator  rbegin()       { return m_Nodes.rbegin();  }
        const_reverse_iterator  rbegin() const { return m_Nodes.rbegin();  }
        const_reverse_iterator crbegin() const { return m_Nodes.crbegin(); }

              reverse_iterator  rend()       { return m_Nodes.rend();  }
        const_reverse_iterator  rend() const { return m_Nodes.rend();  }
        const_reverse_iterator crend() const { return m_Nodes.crend(); }

    };

} // CHDR::Mazes

#endif //CHDR_GRID_HPP