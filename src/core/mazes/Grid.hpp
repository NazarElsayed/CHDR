#ifndef CHDR_GRID_HPP
#define CHDR_GRID_HPP

#include "../types/Coord.hpp"
#include "../types/Node.hpp"
#include "../utils/Utils.hpp"
#include "base/IMaze.hpp"

#include <Debug.hpp>

#include <vector>
#include <array>

namespace CHDR::Mazes {

    /**
     * @tparam Kd Dimensionality of the grid.
     */
    template <const size_t Kd, typename T = uint32_t>
    class Grid : public IMaze<T> {

        using coord_t = Coord<size_t, Kd>;

    public:

        static constexpr auto Rank = Kd;

        static_assert(              Kd > 0U, "Kd must be greater than 0.");
        static_assert(std::is_integral_v<T>, "T must be integral."       );

    private:

        coord_t m_Size;

        std::vector<Node<T>> m_Nodes;

    public:

        constexpr Grid(const coord_t& _size) :
            m_Size(_size),
            m_Nodes(Utils::Product<size_t>(m_Size)) {}

        constexpr Grid(const coord_t& _size, const std::vector<Node<T>>& _nodes) :
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
        constexpr Grid(const Args&... _size, const std::vector<Node<T>>& _nodes) :
            m_Size { _size... },
            m_Nodes(_nodes)
        {
            static_assert(sizeof...(Args) == Rank, "Number of arguments must equal the Grid's rank.");
        }

        [[nodiscard]] constexpr const std::vector<Node<T>>& Nodes() const {
            return m_Nodes;
        }

        constexpr void Nodes(const std::vector<Node<T>>& _value) {
            m_Nodes = _value;
        }

        [[nodiscard]] constexpr const coord_t& Size() const {
            return m_Size;
        }

        template<typename... Args>
        constexpr void Size(const Args&... _value) {
            Size({ _value... });
        }

        constexpr void Size(const coord_t& _value) {
            m_Size = _value;
        }

        template<typename... Args>
        auto GetNeighbours(const Args&... _coord) const {
            return GetNeighbours({ _coord... });
        }

        constexpr auto GetNeighbours(const coord_t& _coord) const {

            std::array<std::pair<bool, coord_t>, Kd * 2U> result;

            for (size_t i = 0U; i < Kd; ++i) {

                coord_t nCoord = _coord;
                --nCoord[i];

                result[i].first  = _coord[i] > 0U && At(nCoord).IsActive();
                result[i].second = nCoord;
            }

            for (size_t i = 0U; i < Kd; ++i) {

                coord_t pCoord = _coord;
                ++pCoord[i];


                result[Kd + i].first  = _coord[i] < m_Size[i] - 1U && At(pCoord).IsActive();
                result[Kd + i].second = pCoord;
            }

            return result;
        }

        constexpr auto GetNeighbours(const size_t& _coord) const {

            std::array<std::pair<bool, coord_t>, Kd * 2U> result;

            const auto c = Utils::ToND<size_t, Kd>(_coord, Size());

            for (size_t i = 0U; i < Kd; ++i) {

                coord_t nCoord = c;
                --nCoord[i];

                result[i].first  = c[i] > 0U && At(nCoord).IsActive();
                result[i].second = nCoord;
            }

            for (size_t i = 0U; i < Kd; ++i) {

                coord_t pCoord = c;
                ++pCoord[i];

                result[Kd + i].first  = c[i] < m_Size[i] - 1U && At(pCoord).IsActive();
                result[Kd + i].second = pCoord;
            }

            return result;
        }

        template<typename... Args>
        [[nodiscard]] constexpr const Node<T>& At(const Args&... _coord) const {
            return At({ _coord... });
        }

        [[nodiscard]] constexpr const Node<T>& At(const coord_t& _coord) const {

            const size_t index = Utils::To1D(_coord, m_Size);

#ifndef NDEBUG
            return m_Nodes.at(index);
#else
            return m_Nodes[index];
#endif // NDEBUG
        }

        template<typename... Args>
        [[nodiscard]] constexpr const Node<T>& At(const size_t& _index) const {

#ifndef NDEBUG
            return m_Nodes.at(_index);
#else
            return m_Nodes[_index];
#endif // NDEBUG
        }

        template<typename... Args>
        [[nodiscard]] constexpr bool Contains(const Args&... _coord) const {
            return Contains({ _coord... });
        }

        [[nodiscard]] constexpr bool Contains(const coord_t _coord) const {

            bool result = true;

            for (size_t i = 0U; i < Kd; ++i) {

                if (_coord[i] >= m_Size[i]) {
                    result = false;

                    break;
                }
            }

            return result;
        }

        [[nodiscard]] constexpr bool Contains(const size_t _coord) const {
            return _coord < Utils::Product<size_t>(m_Size);
        }
    };

} // CHDR::Mazes

#endif //CHDR_GRID_HPP