#ifndef CHDR_ASTAR_HPP
#define CHDR_ASTAR_HPP

#include <cmath>

#include <Debug.hpp>

#include <functional>
#include <list>
#include <queue>
#include <unordered_set>

#include "base/ISolver.hpp"
#include "types/Heap.hpp"

namespace CHDR::Solvers {

    template<typename Tm, const size_t Kd, typename Ts>
    class AStar : public ISolver<Tm> {

        static_assert(std::is_integral_v<Ts> || std::is_floating_point_v<Ts>, "Ts must be either an integral or floating point type");

    private:

        using coord_t = Coord<size_t, Kd>;

        struct ASNode {

            size_t m_Coord;

            Ts m_GScore;
            Ts m_FScore;

            const ASNode* m_Parent;

            [[nodiscard]] constexpr ASNode(const size_t& _coord, const Ts& _gScore, const Ts& _hScore, const ASNode* const _parent) :
                m_Coord (_coord),
                m_GScore(_gScore),
                m_FScore(_gScore + _hScore),
                m_Parent(_parent) {}

            [[nodiscard]] constexpr bool operator == (const ASNode& _node) const { return m_Coord == _node.m_Coord; }
        };

        struct ASNodeCompare {

            [[nodiscard]] constexpr bool operator () (const ASNode& _a, const ASNode& _b) const { return _a.m_FScore > _b.m_FScore; }
        };

    public:

        void Solve(const Mazes::IMaze<Tm>& _maze) override {
            throw std::runtime_error("AStar::Solve(const Mazes::IMaze& _maze) Not implemented!");
        }

        auto Solve(const Mazes::Grid<Kd, Tm>& _maze, const coord_t& _start, const coord_t& _end, Ts (*_h)(const coord_t&, const coord_t&)) const {

            std::vector<coord_t> result;

            const auto s = Utils::To1D(_start, _maze.Size());
            const auto e = Utils::To1D(_end,   _maze.Size());

            std::list<ASNode> buffer;

            std::unordered_set<size_t, std::function<const size_t&(const size_t&)>> closedSet(
                static_cast<size_t>(ceil(_h(_start, _end))),
                [](const size_t& _seed) constexpr -> const size_t& { return _seed; }
            );

            std::priority_queue<ASNode, std::vector<ASNode>, ASNodeCompare> openSet;
            openSet.emplace(s, static_cast<Ts>(0), _h(_start, _end), nullptr);

            while (!openSet.empty()) {

                buffer.emplace_back(std::move(openSet.top()));
                openSet.pop();

                const auto& current = buffer.back();

                if (current.m_Coord != e) {

                    /* SEARCH FOR SOLUTION */

                    closedSet.emplace(current.m_Coord);

                    for (const auto neighbour : _maze.GetActiveNeighbours(current.m_Coord)) {

                        const auto n = Utils::To1D(neighbour, _maze.Size());

                        // Check if node is not already visited:
                        if (closedSet.find(n) == closedSet.end()) {

                            // Add node to openSet.
                            openSet.emplace(n, current.m_GScore + static_cast<Ts>(1), _h(neighbour, _end), &current);
                        }
                    }
                }
                else {

                    /* SOLUTION REACHED */

                    // Free data which is no longer relevant:
                    std::priority_queue<ASNode, std::vector<ASNode>, ASNodeCompare>().swap(openSet);
                    closedSet.clear();

                    // Recurse from end node to start node, inserting into a result buffer:
                    result.reserve(current.m_GScore);

                    for (const auto* temp = &current; temp->m_Parent != nullptr; temp = temp->m_Parent) {
                        result.push_back(Utils::ToND(temp->m_Coord, _maze.Size()));
                    }

                    // Reverse the buffer:
                    std::reverse(result.begin(), result.end());
                }
            }

            //PrintPath(result);

            return result;
        }

        static bool HasSolution(const Mazes::Grid<Kd, Tm>& _maze, const coord_t& _start, const coord_t& _end, size_t _capacity = 1U) {

	        bool result = false;

            const auto s = Utils::To1D(_start, _maze.Size());
            const auto e = Utils::To1D(_end,   _maze.Size());

            if (_maze.Contains(s) &&
                _maze.Contains(e) &&
                _maze.At(s).IsActive() &&
                _maze.At(e).IsActive()
            ) {
                std::queue<size_t> openSet;
                openSet.emplace(s);

                std::unordered_set<size_t> closedSet;
                closedSet.reserve(_capacity);
                closedSet.emplace(s);

                while (!openSet.empty()) {

                    for (size_t i = 0U; i < openSet.size(); ++i) {

                        const auto curr = openSet.front();
                        openSet.pop();

                        for (const auto neighbour : _maze.GetActiveNeighbours(curr)) {

                            const auto n = Utils::To1D(neighbour, _maze.Size());

                            const auto [iter, inserted] = closedSet.emplace(n);
                            if (inserted) {

                                if (n == e) {
                                    result = true;

                                    goto NestedBreak;
                                }

                                openSet.emplace(n);
                            }
                        }
                    }
                }
            }

NestedBreak:
            return result;
        }

        constexpr void PrintPath(std::vector<coord_t>& _path) const {

            for (const auto& coord : _path) {

                std::ostringstream oss;
                oss << "(";

                for (size_t i = 0U; i < Kd; ++i) {
                    oss << coord[i] << (i < Kd - 1U ? ", " : "");
                }

                oss << ")";

                Debug::Log(oss.str(), Info);
            }
        }

    };
} // CHDR::Solvers

#endif //CHDR_ASTAR_HPP