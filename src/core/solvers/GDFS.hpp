/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_GDFS_HPP
#define CHDR_GDFS_HPP

#include <functional>
#include <stack>

#include "base/ISolver.hpp"
#include "mazes/base/IMaze.hpp"
#include "types/ExistenceSet.hpp"
#include "utils/Utils.hpp"
#include "mazes/Grid.hpp"

namespace CHDR::Solvers {

    template<typename Tm, const size_t Kd>
    class GDFS final : public ISolver<Tm> {

        struct GDFSNode final {

            size_t m_Coord;

            std::shared_ptr<const GDFSNode> m_Parent;

            [[nodiscard]] constexpr GDFSNode() :
                m_Coord(-1U),
                m_Parent(nullptr) {}

            [[nodiscard]] constexpr GDFSNode(const size_t &_coord, const std::shared_ptr<const GDFSNode>& _parent) :
                m_Coord(_coord),
                m_Parent(std::move(_parent)) {}

            ~GDFSNode() {

                while (m_Parent && static_cast<unsigned>(m_Parent.use_count()) < 2U) {
                    m_Parent = std::move(m_Parent->m_Parent);
                }
            }
        };

    private:

        using coord_t = Coord<size_t, Kd>;

    public:

        void Solve(const Mazes::IMaze<Tm>& _maze) override {

            (void)_maze; // Suppress unused variable warning.

            throw std::runtime_error("GDFS::Solve(const Mazes::IMaze& _maze): Not implemented!");
        }

        auto Solve(const Mazes::Grid<Kd, Tm>& _maze, const coord_t& _start, const coord_t& _end, size_t _capacity = 0U) {

            std::vector<coord_t> result;

            const auto s = Utils::To1D(_start, _maze.Size());
            const auto e = Utils::To1D(_end,   _maze.Size());

            if (_maze.Contains(s) &&
                _maze.Contains(e) &&
                _maze.At(s).IsActive() &&
                _maze.At(e).IsActive()
            ) {

                _capacity = std::max(_capacity, std::max(s, e));

                auto sequence = std::vector<GDFSNode>(_capacity);
                std::stack<GDFSNode, std::vector<GDFSNode>> openSet(std::move(sequence));
                openSet.emplace(s, nullptr);

                ExistenceSet closedSet({ s }, _capacity);

                while (!openSet.empty()) { // SEARCH FOR SOLUTION...

                    for (size_t i = 0U; i < openSet.size(); ++i) {

                        const GDFSNode current(std::move(openSet.top()));
                        openSet.pop();

                        if (current.m_Coord != e) {

                            if (closedSet.Capacity() > current.m_Coord) {
                                closedSet.Reserve(std::min(_capacity * ((current.m_Coord % _capacity) + 1U), Utils::Product<size_t>(_maze.Size())));
                            }
                            closedSet.Add(current.m_Coord);

                            for (const auto& neighbour: _maze.GetNeighbours(current.m_Coord)) {

                                if (const auto& [nActive, nCoord] = neighbour; nActive) {

                                    const auto n = Utils::To1D(nCoord, _maze.Size());

                                    // Check if node is not already visited:
                                    if (!closedSet.Contains(n)) {

                                        // Add to dupe list:
                                        if (closedSet.Capacity() > n) {
                                            closedSet.Reserve(std::min(_capacity * ((n % _capacity) + 1U), Utils::Product<size_t>(_maze.Size())));
                                        }
                                        closedSet.Add(n);

                                        // Create a parent node and transfer ownership of 'current' to it. Note: 'current' is now moved!
                                        openSet.emplace(n, std::make_shared<GDFSNode>(std::move(current)));
                                    }
                                }
                            }
                        }
                        else { // SOLUTION REACHED ...

                            // Free data which is no longer relevant:
                            std::stack<GDFSNode, std::vector<GDFSNode>> empty;
                            std::swap(openSet, empty);

                            closedSet.Clear();

                            // Recurse from end node to start node, inserting into a result buffer:
                            result.reserve(_capacity);
                            result.emplace_back(Utils::ToND(current.m_Coord, _maze.Size()));

                            if (current.m_Parent != nullptr) {

                                for (auto& item = current.m_Parent; item->m_Parent != nullptr;) {
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

            return result;
        }

    };

} // CHDR::Solvers

#endif //CHDR_GDFS_HPP