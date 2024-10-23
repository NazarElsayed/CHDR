/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_GDFS_HPP
#define CHDR_GDFS_HPP

#include <stack>

#include "base/ISolver.hpp"
#include "mazes/base/IMaze.hpp"
#include "types/ExistenceSet.hpp"
#include "utils/Utils.hpp"
#include "mazes/Grid.hpp"

namespace CHDR::Solvers {

    template<typename Tm, const size_t Kd>
    class GDFS final {

        struct GDFSNode final {

            size_t m_Coord;

            std::shared_ptr<const GDFSNode> m_Parent;

            /**
             * @brief Constructs an uninitialized ASNode.
             *
             * This constructor creates an ASNode with uninitialized members.
             */
            [[nodiscard]] constexpr GDFSNode() {} // NOLINT(*-pro-type-member-init, *-use-equals-default)

            [[nodiscard]] constexpr GDFSNode(const size_t& _coord, GDFSNode&& _parent) :
                m_Coord(_coord)
            {
                m_Parent = std::make_shared<const GDFSNode>(std::move(_parent));
            }

            ~GDFSNode() {

//                while (m_Parent && m_Parent.unique()) {
//                    m_Parent = std::move(m_Parent->m_Parent);
//                }

                Expunge_Recursive(m_Parent);
            }

            void Expunge_Recursive(std::shared_ptr<const GDFSNode>& _node) {
                if (_node && _node.unique()) {
                    _node = std::move(_node->m_Parent);
                    Expunge_Recursive(_node);
                }
            }
        };

    private:

        using coord_t = Coord<size_t, Kd>;

    public:

        auto Solve(const Mazes::Grid<Kd, Tm>& _maze, const coord_t& _start, const coord_t& _end, size_t _capacity = 0U) {

            std::vector<coord_t> result;

            const auto s = Utils::To1D(_start, _maze.Size());
            const auto e = Utils::To1D(_end,   _maze.Size());

            if (_maze.Contains(s) &&
                _maze.Contains(e) &&
                _maze.At(s).IsActive() &&
                _maze.At(e).IsActive()
            ) {

                if (s != e) {

                    const auto count = _maze.Count();

                    _capacity = std::max(_capacity, std::max(s, e));

                    auto sequence = std::vector<GDFSNode>(_capacity);
                    std::stack<GDFSNode, std::vector<GDFSNode>> open(std::move(sequence));
                    open.emplace(s, nullptr);

                    ExistenceSet closed({ s }, _capacity);

                    while (!open.empty()) { // SEARCH FOR SOLUTION...

                        for (size_t i = 0U; i < open.size(); ++i) {

                            auto curr = open.PopTop();

                            if (curr.m_Coord != e) {

                                if (closed.Capacity() > curr.m_Coord) {
                                    closed.Reserve(std::min(_capacity * ((curr.m_Coord % _capacity) + 1U), count));
                                }
                                closed.Add(curr.m_Coord);

                                for (const auto& neighbour: _maze.GetNeighbours(curr.m_Coord)) {

                                    if (const auto& [nActive, nCoord] = neighbour; nActive) {

                                        const auto n = Utils::To1D(nCoord, _maze.Size());

                                        // Check if node is not already visited:
                                        if (!closed.Contains(n)) {

                                            if (closed.Capacity() > n) {
                                                closed.Reserve(std::min(_capacity * ((n % _capacity) + 1U), count));
                                            }
                                            closed.Add(n);

                                            // Create a parent node and transfer ownership of 'current' to it. Note: 'current' is now moved!
                                            open.emplace(n, std::move(curr));
                                        }
                                    }
                                }
                            }
                            else { // SOLUTION REACHED ...

                                // Free data which is no longer relevant:
                                std::stack<GDFSNode, std::vector<GDFSNode>> empty;
                                std::swap(open, empty);

                                closed.Clear(); closed.Trim();

                                // Recurse from end node to start node, inserting into a result buffer:
                                result.reserve(_capacity);
                                result.emplace_back(Utils::ToND(curr.m_Coord, _maze.Size()));

                                if (curr.m_Parent != nullptr) {

                                    for (auto& item = curr.m_Parent; item->m_Parent != nullptr;) {
                                        result.emplace_back(Utils::ToND(item->m_Coord, _maze.Size()));

                                        auto oldItem = item;
                                        item = item->m_Parent;
                                        oldItem.reset();
                                    }
                                }

                                // Reverse the result:
                                std::reverse(result.begin(), result.end());

                                break;
                            }
                        }
                    }
                }
                else {
                    result.emplace_back(_end);
                }
            }

            return result;
        }

    };

} // CHDR::Solvers

#endif //CHDR_GDFS_HPP