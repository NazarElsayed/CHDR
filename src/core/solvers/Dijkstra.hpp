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

    template<typename Tm, const size_t Kd, typename Ts, typename Ti>
    class [[maybe_unused]] Dijkstra final {

        static_assert(std::is_integral_v<Ti>, "Ti must be an integral type.");

    private:

        using coord_t = Coord<Ti, Kd>;

        struct DijkstraNode final {

            Ti m_Index;

            Ts m_GScore;
            Ts m_FScore;

            std::shared_ptr<const DijkstraNode> m_Parent;

            /**
             * @brief Constructs an uninitialized ASNode.
             *
             * This constructor creates an ASNode with uninitialized members.
             */
            [[nodiscard]] constexpr DijkstraNode() {} // NOLINT(*-pro-type-member-init, *-use-equals-default)

            [[nodiscard]] constexpr DijkstraNode(const Ti &_index, const Ts &_gScore, const Ts &_hScore, const std::shared_ptr<const DijkstraNode>& _parent) :
                m_Index(_index),
                m_GScore(_gScore),
                m_FScore(_gScore + _hScore),
                m_Parent(std::move(_parent)) {}

            ~DijkstraNode() { // NOLINT(*-use-equals-default)

//                while (m_Parent && static_cast<unsigned>(m_Parent.use_count()) < 2U) {
//                    m_Parent = std::move(m_Parent->m_Parent);
//                }

                Expunge_Recursive(m_Parent);
            }

            void Expunge_Recursive(std::shared_ptr<const DijkstraNode>& _node) {
                if (_node && _node.unique()) {
                    _node = std::move(_node->m_Parent);
                    Expunge_Recursive(_node);
                }
            }

            [[nodiscard]] constexpr bool operator == (const DijkstraNode& _node) const { return m_Index == _node.m_Index; }

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

        [[maybe_unused]]
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