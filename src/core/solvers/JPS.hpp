/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_JPS_HPP
#define CHDR_JPS_HPP
#include <map>

namespace CHDR::Solvers {

    template<typename Tm, const size_t Kd, typename Ts, typename Ti>
    class [[maybe_unused]] JPS final {

        static_assert(std::is_integral_v<Ts> || std::is_floating_point_v<Ts>, "Ts must be either an integral or floating point type");
        static_assert(std::is_integral_v<Ti>, "Ti must be an integral type.");

    private:

        using coord_t = Coord<Ti, Kd>;

        /*
        const std::array<int, 8> s_rotateL { 5, 6, 7,
                                             3,    4,
                                             0, 1, 2 };

        const std::array<int, 8> s_rotate2 { 7, 4, 2,
                                             6,    1,
                                             5, 3, 0 };

        const std::array<int, 8> s_rotateR { 2, 1, 0,
                                             4,    3,
                                             7, 6, 5 };


        const std::map<std::array<int, 2>, std::array<int, 8>> rotationMap {
            { { 1,  0}, { 0, 1, 2, 3, 4, 5, 6, 7 } },
            { { 1,  1}, { 0, 1, 2, 3, 4, 5, 6, 7 } },
            { { 0,  1}, s_rotateL },
            { {-1,  1}, s_rotateL },
            { {-1,  0}, s_rotate2 },
            { {-1, -1}, s_rotate2 },
            { { 0, -1}, s_rotateR },
            { { 1, -1}, s_rotateR },
        };*/

        /*
         * 0 1 2
         * 3   4
         * 5 6 7
         */

        const std::array<int, 8> s_rotateL { 2, 4, 7,
                                             1,    6,
                                             0, 3, 5 };

        const std::array<int, 8> s_rotate2 { 7, 6, 5,
                                             4,    3,
                                             2, 1, 0 };

        const std::array<int, 8> s_rotateR { 5, 3, 0,
                                             6,    1,
                                             7, 4, 2 };


        const std::map<std::array<int, 2>, std::array<int, 8>> rotationMap {
                { { 0,  0}, { 0, 1, 2, 3, 4, 5, 6, 7 } },
                { { 1,  0}, { 0, 1, 2, 3, 4, 5, 6, 7 } },
                { { 1,  1}, { 0, 1, 2, 3, 4, 5, 6, 7 } },
                { { 0,  1}, s_rotateL },
                { {-1,  1}, s_rotateL },
                { {-1,  0}, s_rotate2 },
                { {-1, -1}, s_rotate2 },
                { { 0, -1}, s_rotateR },
                { { 1, -1}, s_rotateR }
            };

        struct JPSNode final {

            Ti m_Index{};

            std::array<int, 2> m_Direction;
            Ts m_GScore;
            Ts m_FScore;

            const JPSNode* RESTRICT m_Parent;

            [[nodiscard]] constexpr JPSNode() = default;

            [[nodiscard]] constexpr JPSNode(const Ti& _index, const std::array<int, 2> _direction, const Ts& _gScore, const Ts& _hScore, const JPSNode* RESTRICT const _parent) :
                m_Index(_index),
                m_Direction(_direction),
                m_GScore(_gScore),
                m_FScore(_gScore + _hScore),
                m_Parent(std::move(_parent)) {}

            [[nodiscard]] constexpr bool operator == (const JPSNode& _node) const { return m_Index == _node.m_Index; }

            struct Max {

                [[nodiscard]] constexpr bool operator () (const JPSNode& _a, const JPSNode& _b) const {

                    return _a.m_FScore == _b.m_FScore ?
                        _a.m_GScore > _b.m_GScore :
                        _a.m_FScore > _b.m_FScore;
                }
            };

            struct Min {

                [[nodiscard]] constexpr bool operator () (const JPSNode& _a, const JPSNode& _b) const {

                    return _a.m_FScore == _b.m_FScore ?
                        _a.m_GScore < _b.m_GScore :
                        _a.m_FScore < _b.m_FScore;
                }
            };
        };

        /*std::array<int, 2> current = { _current[0], _current[1] };
        Debug::Log("Direction: {" + std::to_string(_direction[0]) + ", " + std::to_string(_direction[1]) + "}");

        for (int i = 0; i < neighbours.size(); i++) {
            std::array<int, 2> coord = { neighbours[i].second[0], neighbours[i].second[1] };
            Debug::Log(std::to_string(coord[0] - current[0]) + ", " + std::to_string(coord[1] - current[1]));
        }

        Debug::Log("");*/

        /*
         * 0 1 2
         * 3   4
         * 5 6 7
         */

        /*
         * 0 3 5
         * 1   6
         * 2 4 7
         */

        template <typename T>
        static int Sign(T val) {
            return (static_cast<T>(0) < val) - (val < static_cast<T>(0));
        }

        std::vector<coord_t> FindJumpPoints(const Mazes::Grid<Kd, Tm>& _maze, const coord_t& _current, const std::array<int, 2> _direction, const coord_t& _end) const {

            std::vector<coord_t> result;

            const auto neighbours = _maze. template GetNeighbours<true>(_current);
            const auto map = rotationMap.at(_direction);

            if (_direction[0] == 0 && _direction[1] == 0) { // Start Node
                for (auto& neighbour : neighbours) {
                    if (neighbour.first) {
                        if (const auto& [nActive, nCoord] = Jump(_maze, neighbour.second, _current, _end); nActive) {
                            result.emplace_back(nCoord);
                        }
                    }
                }
            }
            else if (_direction[0] == 0 || _direction[1] == 0) { // Straight Direction

                // Check and expand forced neighbours:
                if (neighbours[map[2]].first && !neighbours[map[1]].first) {
                    if (const auto& [nActive, nCoord] = Jump(_maze, neighbours[map[2]].second, _current, _end); nActive) {
                        result.emplace_back(nCoord);
                    }
                }
                if (neighbours[map[7]].first && !neighbours[map[6]].first) {
                    if (const auto& [nActive, nCoord] = Jump(_maze, neighbours[map[7]].second, _current, _end); nActive) {
                        result.emplace_back(nCoord);
                    }
                }

                // Expand natural neighbours
                if (neighbours[map[4]].first) {
                    if (const auto& [nActive, nCoord] = Jump(_maze, neighbours[map[4]].second, _direction, _end); nActive) {
                        result.emplace_back(nCoord);
                    }
                }
            }
            else { // Diagonal Direction

                // Check diagonal is not blocked:
                if (neighbours[map[1]].first ||
                    neighbours[map[3]].first) {

                    // Check and expand forced neighbours:
                    if (neighbours[map[2]].first && !neighbours[map[1]].first) {
                        if (const auto& [nActive, nCoord] = Jump(_maze, neighbours[map[2]].second, _current, _end); nActive) {
                            result.emplace_back(nCoord);
                        }
                    }
                    if (neighbours[map[5]].first && !neighbours[map[3]].first) {
                        if (const auto& [nActive, nCoord] = Jump(_maze, neighbours[map[5]].second, _current, _end); nActive) {
                            result.emplace_back(nCoord);
                        }
                    }

                    // Expand natural neighbours:
                    if (neighbours[map[4]].first) {
                        if (const auto& [nActive, nCoord] = Jump(_maze, neighbours[map[4]].second, _current, _end); nActive) {
                            result.emplace_back(nCoord);
                        }
                    }

                    if (neighbours[map[6]].first) {
                        if (const auto& [nActive, nCoord] = Jump(_maze, neighbours[map[6]].second, _current, _end); nActive) {
                            result.emplace_back(nCoord);
                        }
                    }

                    if (neighbours[map[7]].first) {
                        if (const auto& [nActive, nCoord] = Jump(_maze, neighbours[map[7]].second, _direction, _end); nActive) {
                            result.emplace_back(nCoord);
                        }
                    }
                }
            }

            return result;
        }

        [[nodiscard]]
        std::pair<bool, coord_t> Jump(const Mazes::Grid<Kd, Tm>& _maze, const coord_t& _current, const coord_t& _previous, const coord_t& _end) const {
            const std::array<int, 2> direction { Sign(static_cast<int>(_current[0]) - static_cast<int>(_previous[0])) ,
                                                 Sign(static_cast<int>(_current[1]) - static_cast<int>(_previous[1])) };

            return Jump(_maze, _current, direction, _end);
        }

        [[nodiscard]]
        std::pair<bool, coord_t> Jump(const Mazes::Grid<Kd, Tm>& _maze, const coord_t& _current, const std::array<int, 2>& _direction, const coord_t& _end) const {

            std::pair<bool, coord_t> result { false, _current };

            const auto neighbours = _maze. template GetNeighbours<true>(_current);
            const auto map = rotationMap.at(_direction);

            if (_direction[0] == 0 || _direction[1] == 0) { // Straight Direction

                if (_current == _end) {
                    result.first = true;
                }
                else {

                    // Check for forced neighbours:
                    if (neighbours[map[2]].first && !neighbours[map[1]].first ||
                        neighbours[map[7]].first && !neighbours[map[6]].first) {

                        result.first = true;
                    }
                    // Expand natural neighbours:
                    else {
                        if (neighbours[map[4]].first) {
                            result = Jump(_maze, neighbours[map[4]].second, _direction, _end);
                        }
                    }
                }
            }
            else { // Diagonal Direction

                // Check diagonal is not blocked:
                if (neighbours[map[1]].first ||
                    neighbours[map[3]].first) {

                    if (_current == _end) {
                        result.first = true;
                    }
                    else {

                        // Check for forced neighbours:
                        if (neighbours[map[2]].first && !neighbours[map[1]].first ||
                            neighbours[map[5]].first && !neighbours[map[3]].first) {

                            result.first = true;
                        }
                        // Expand natural neighbours:
                        else {
                            if (neighbours[map[4]].first) {
                                result.first = Jump(_maze, neighbours[map[4]].second, _current, _end).first;
                            }

                            if (!result.first) {
                                if (neighbours[map[6]].first) {
                                    result.first = Jump(_maze, neighbours[map[6]].second, _current, _end).first;
                                }
                            }

                            if (!result.first) {
                                if (neighbours[map[7]].first) {
                                    result = Jump(_maze, neighbours[map[7]].second, _direction, _end);
                                }
                            }
                        }
                    }
                }
            }

            return result;
        }

        auto SolveHeap(const Mazes::Grid<Kd, Tm>& _maze, const coord_t& _start, const coord_t& _end, Ts (*_h)(const coord_t&, const coord_t&), const Ts& _weight = 1, size_t _capacity = 0U) const {

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
                    const auto size = _maze.Size();

                    _capacity = std::max(_capacity, std::max(s, e));

                    ExistenceSet<LowMemoryUsage> closed({ s }, _capacity);

                    Heap<JPSNode, 2U, typename JPSNode::Max> open(_capacity / 8U);
                    open.Emplace({ s, {0, 0}, static_cast<Ts>(0), _h(_start, _end), nullptr });

                    StableForwardBuf<JPSNode> buf;

                    while (!open.Empty()) {

                        auto curr = open.PopTop();

                        if (curr.m_Index != e) { // SEARCH FOR SOLUTION...

                            if (closed.Capacity() < curr.m_Index) {
                                closed.Reserve(std::min(_capacity * ((curr.m_Index % _capacity) + 1U), count));
                            }
                            closed.Add(curr.m_Index);

                            auto coord = Utils::ToND(curr.m_Index, size);
                            auto successors = FindJumpPoints(_maze, coord, curr.m_Direction, _end);

                            for (const auto& successor : successors) {

                                const auto n = Utils::To1D(successor, size);
                                if (!closed.Contains(n)) {

                                    // Check if node is not already visited:
                                    if (closed.Capacity() < n) {
                                        closed.Reserve(std::min(_capacity * ((n % _capacity) + 1U), count));
                                    }

                                    closed.Add(n);

                                    const std::array<int, 2> direction { Sign(static_cast<int>(successor[0]) - static_cast<int>(coord[0])) ,
                                                                         Sign(static_cast<int>(successor[1]) - static_cast<int>(coord[1])) };
                                    // Create a parent node and transfer ownership of 'current' to it. Note: 'current' is now moved!
                                    open.Emplace({ n, direction, curr.m_GScore + static_cast<Ts>(1), _h(successor, _end) * _weight, &buf.Emplace(std::move(curr)) });
                                }
                            }
                        }
                        else { // SOLUTION REACHED ...

                            // Recurse from end node to start node, inserting into a result buffer:
                            result.reserve(curr.m_GScore);
                            for (const auto* temp = &curr; temp->m_Parent != nullptr; temp = temp->m_Parent) {
                                result.emplace_back(Utils::ToND(temp->m_Index, _maze.Size()));
                            }

                            // Reverse the result:
                            std::reverse(result.begin(), result.end());

                            break;
                        }
                    }
                }
                else {
                    result.emplace_back(_end);
                }
            }

            return result;
        }

        template <size_t StackSize>
        auto SolveLinear(const Mazes::Grid<Kd, Tm>& _maze, const coord_t& _start, const coord_t& _end, Ts (*_h)(const coord_t&, const coord_t&), const Ts& _weight = 1, size_t _capacity = 0U) const {

            std::vector<coord_t> result;

            /*
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

                    ExistenceSet<LowMemoryUsage> closed({ s }, _capacity);

                    std::vector<JPSNode, StackAllocator<JPSNode, StackSize>> open;
                    open.reserve(StackSize);
                    open.push_back({ s, static_cast<Ts>(0), _h(_start, _end), nullptr });

                    StableForwardBuf<JPSNode, StackSize / 2U> buf;

                    while (!open.empty()) {

                        const auto top = std::min_element(open.begin(), open.end(), typename JPSNode::Min());
                        auto curr(std::move(*top));
                        open.erase(top);

                        if (curr.m_Index != e) { // SEARCH FOR SOLUTION...

                            if (closed.Capacity() < curr.m_Index) {
                                closed.Reserve(std::min(_capacity * ((curr.m_Index % _capacity) + 1U), count));
                            }
                            closed.Add(curr.m_Index);

                            for (const auto& neighbour : _maze.GetNeighbours(curr.m_Index)) {

                                if (const auto& [nActive, nCoord] = neighbour; nActive) {

                                    const auto n = Utils::To1D(nCoord, _maze.Size());

                                    // Check if node is not already visited:
                                    if (!closed.Contains(n)) {

                                        if (closed.Capacity() < n) {
                                            closed.Reserve(std::min(_capacity * ((n % _capacity) + 1U), count));
                                        }
                                        closed.Add(n);

                                        // Create a parent node and transfer ownership of 'current' to it. Note: 'current' is now moved!
                                        open.push_back({n, curr.m_GScore + static_cast<Ts>(1), _h(nCoord, _end) * _weight, &buf.Emplace(std::move(curr)) });
                                    }
                                }
                            }
                        }
                        else { // SOLUTION REACHED ...

                            // Recurse from end node to start node, inserting into a result buffer:
                            result.reserve(curr.m_GScore);

                            for (const auto* temp = &curr; temp->m_Parent != nullptr; temp = temp->m_Parent) {
                                result.emplace_back(Utils::ToND(temp->m_Index, _maze.Size()));
                            }

                            // Reverse the result:
                            std::reverse(result.begin(), result.end());

                            break;
                        }
                    }
                }
                else {
                    result.emplace_back(_end);
                }
            }
            */

            return result;
        }

    public:

        [[maybe_unused]]
        auto Solve(const Mazes::Grid<Kd, Tm>& _maze, const coord_t& _start, const coord_t& _end, Ts (*_h)(const coord_t&, const coord_t&), const Ts& _weight = 1, size_t _capacity = 0U) const {
            /*
             * Determine whether to solve using a linear search or constant-time
             * method based on which is more efficient given the maze's size.
             */

            constexpr size_t LMAX = 256U;

            std::vector<coord_t> result;

            const auto count = _maze.Count();

            if (count <= 64U) {
                result = SolveLinear<32U>(_maze, _start, _end, _h, _weight, _capacity);
            }
            else if (count <= 128U) {
                result = SolveLinear<64U>(_maze, _start, _end, _h, _weight, _capacity);
            }
            else if (count <= LMAX) {
                result = SolveLinear<LMAX / 2U>(_maze, _start, _end, _h, _weight, _capacity);
            }
            else {
                result = SolveHeap(_maze, _start, _end, _h, _weight, _capacity);
            }

            return result;
        }
    };

} // CHDR::Solvers

#endif //CHDR_JPS_HPP