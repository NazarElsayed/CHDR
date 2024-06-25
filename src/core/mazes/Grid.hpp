#ifndef CHDR_GRID_HPP
#define CHDR_GRID_HPP

#include "../Coord.hpp"
#include "../Node.hpp"
#include "../Utils.hpp"
#include "base/IMaze.hpp"

#include <Debug.hpp>

#include <vector>

namespace CHDR::Mazes {

    /**
     * @tparam Kd Dimensionality of the grid.
     */
    template <size_t Kd, typename T = uint32_t>
    class Grid : public IMaze {

        using coord_t = Coord<size_t, Kd>;

    public:

        static constexpr auto Rank = Kd;

        static_assert(Kd > 0, "Kd must be greater than 0.");
        static_assert(std::is_integral_v<T>, "T must be integral.");

    private:

        coord_t m_Size;

        std::vector<Node<T>> m_Nodes;

    public:

        constexpr Grid(const coord_t& _size) {
            m_Size = _size;

            m_Nodes.resize(Utils::Product<std::size_t>(m_Size));
        }

        template <typename... Args>
        constexpr Grid(const Args&... _size) {

            static_assert(sizeof...(Args) == Rank, "Number of arguments must equal the Grid's rank.");

            m_Size = { _size... };

            m_Nodes.resize(Utils::Product<std::size_t>(m_Size));
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

        constexpr void Size(const coord_t& _value) {
            m_Size = _value;
        }

        template<typename... Args>
        [[nodiscard]] constexpr Node<T>& At(const Args&... _coord) {

            const size_t index = Utils::To1D({ _coord... }, m_Size);

#ifndef NDEBUG
            return m_Nodes.at(index);
#else
            return m_Nodes[index];
#endif // NDEBUG
        }

        [[nodiscard]] constexpr Node<T>& At(const coord_t& _coord) {

            const size_t index = Utils::To1D(_coord, m_Size);

#ifndef NDEBUG
            return m_Nodes.at(index);
#else
            return m_Nodes[index];
#endif // NDEBUG
        }

    };

} // CHDR::Mazes

#endif //CHDR_GRID_HPP