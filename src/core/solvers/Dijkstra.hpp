/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_DIJKSTRA_HPP
#define CHDR_DIJKSTRA_HPP

#include <memory>

#include "base/ISolver.hpp"
#include "mazes/base/IMaze.hpp"
#include "types/Heap.hpp"
#include "mazes/Grid.hpp"

namespace CHDR::Solvers {

    template<typename Tm, const size_t Kd, typename Ts>
    class Dijkstra final {

    private:

        using coord_t = Coord<size_t, Kd>;

        struct DijkstraNode final {

            size_t m_Coord;

            Ts m_GScore;
            Ts m_FScore;

            std::shared_ptr<const DijkstraNode> m_Parent;

            /**
             * @brief Constructs an uninitialized ASNode.
             *
             * This constructor creates an ASNode with uninitialized members.
             */
            [[nodiscard]] constexpr DijkstraNode() {} // NOLINT(*-pro-type-member-init, *-use-equals-default)

            [[nodiscard]] constexpr DijkstraNode(const size_t &_coord, const Ts &_gScore, const Ts &_hScore, const std::shared_ptr<const DijkstraNode>& _parent) :
                m_Coord(_coord),
                m_GScore(_gScore),
                m_FScore(_gScore + _hScore),
                m_Parent(std::move(_parent)) {}

            ~DijkstraNode() {

                while (m_Parent && static_cast<unsigned>(m_Parent.use_count()) < 2U) {
                    m_Parent = std::move(m_Parent->m_Parent);
                }
            }

            [[nodiscard]] constexpr bool operator == (const DijkstraNode& _node) const { return m_Coord == _node.m_Coord; }

            struct Max {

                [[nodiscard]] constexpr bool operator () (const DijkstraNode& _a, const DijkstraNode& _b) const {

                    bool result{};

                    if (_a.m_FScore == _b.m_FScore) {
                        result = _a.m_GScore > _b.m_GScore;
                    }
                    else {
                        result = _a.m_FScore > _b.m_FScore;
                    }

                    return result;
                }
            };
        };

    public:

        void Solve(const Mazes::Grid<Kd, Tm>& _maze, const coord_t& _start, const coord_t& _end, Ts (*_h)(const coord_t&, const coord_t&), size_t _capacity = 0U) {

            (void)_maze; // Suppress unused variable warnings.
            (void)_start;
            (void)_end;
            (void)_h;
            (void)_capacity;

            throw std::runtime_error("Djikstra::Solve(const Mazes::Grid<Kd, Tm>& _maze, const coord_t& _start, const coord_t& _end, Ts (*_h)(const coord_t&, const coord_t&), size_t _capacity = 0U): Not implemented!");
        }

    };

} // CHDR::Solvers

#endif //CHDR_DJIKSTRA_HPP