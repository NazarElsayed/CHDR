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

#ifdef __SSE2__

#include <emmintrin.h> // For SSE2 operations.

#endif // __SSE2__

namespace CHDR::Solvers {

    template<typename Tm, const size_t Kd, typename Ts = float>
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

        auto Solve(const Mazes::Grid<Kd, Tm>& _maze, const coord_t& _start, const coord_t& _end, Ts (*_h)(const coord_t&, const coord_t&) = EuclideanDistance) const {

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

#ifdef __SSE2__

        [[nodiscard]] static constexpr Ts simd_sub_64bit_128(const coord_t& _A, const coord_t& _B) {

            const auto regA = _mm_load_si128(reinterpret_cast<__m128i const*>(&_A));
            const auto regB = _mm_load_si128(reinterpret_cast<__m128i const*>(&_B));

            const auto   notB = _mm_xor_si128(regB, _mm_set1_epi64x(-1)); // bitwise not
            const auto minusB = _mm_add_epi64(notB, _mm_set1_epi64x( 1)); // add 1

            // Perform A - B by computing A + (-B):
            const auto sub = _mm_add_epi64(regA, minusB);

            __m128i resultOut{};
            _mm_store_si128(&resultOut, sub);

            const auto* const output = reinterpret_cast<size_t*>(&resultOut);
            return static_cast<Ts>(output[0U] + output[1U]);
        }

        [[nodiscard]] static constexpr Ts simd_sub_64bit_128(
            const size_t& _A0, const size_t& _A1,
            const size_t& _B0, const size_t& _B1
        ) {

            const auto regA = _mm_set_epi64x(_A0, _A1);
            const auto regB = _mm_set_epi64x(_B0, _B1);

            const auto   notB = _mm_xor_si128(regB, _mm_set1_epi64x(-1)); // bitwise not
            const auto minusB = _mm_add_epi64(notB, _mm_set1_epi64x( 1)); // add 1

            // Perform A - B by computing A + (-B):
            const auto sub = _mm_add_epi64(regA, minusB);

            __m128i resultOut{};
            _mm_store_si128(&resultOut, sub);

            const auto* const output = reinterpret_cast<size_t*>(&resultOut);
            return static_cast<Ts>(output[0U] + output[1U]);
        }

        [[nodiscard]] static constexpr Ts simd_sub_32_bit_128(const coord_t& _A, const coord_t& _B) {

            // Load the coordinates into SSE registers:
            const auto regA = _mm_load_si128(reinterpret_cast<__m128i const*>(&_A));
            const auto regB = _mm_load_si128(reinterpret_cast<__m128i const*>(&_B));

            // Perform A - B:
            const auto sub = _mm_sub_epi32(regA, regB);

            __m128i resultOut{};
            _mm_store_si128(&resultOut, sub);

            const auto* const output = reinterpret_cast<size_t*>(&resultOut);
            return static_cast<Ts>(output[0U] + output[1U] + output[2U] + output[3U]);
        }

        [[nodiscard]] static constexpr Ts simd_sub_32_bit_128(
            const size_t& _A0, const size_t& _A1, const size_t& _A2, const size_t& _A3,
            const size_t& _B0, const size_t& _B1, const size_t& _B2, const size_t& _B3
        ) {

            const auto regA = _mm_set_epi32(_A3, _A2, _A1, _A0); // Load as { A3, A2, A1, A0 }
            const auto regB = _mm_set_epi32(_B3, _B2, _B1, _B0); // Load as { A3, B2, B1, B0 }

            // Perform A - B:
            const auto sub = _mm_sub_epi32(regA, regB);

            __m128i resultOut{};
            _mm_store_si128(&resultOut, sub);

            const auto* const output = reinterpret_cast<size_t*>(&resultOut);
            return static_cast<Ts>(output[0U] + output[1U] + output[2U] + output[3U]);
        }

#endif //__SSE2__

#pragma region Heuristics

        /**
         * @brief Computes the Euclidean distance between two nodes.
         *
         * @tparam Kd The number of dimensions of the nodes.
         * @param _A The first node.
         * @param _B The second node.
         * @return The Euclidean distance between _A and _B.
         */
        [[nodiscard]] static constexpr auto EuclideanDistance(const coord_t& _A, const coord_t& _B) {
            return sqrtf(SqrEuclideanDistance(_A, _B));
        }

        /*
         * @brief Computes the squared Euclidean distance between two nodes.
         *
         * @tparam Kd The number of dimensions of the nodes.
         * @param _A The first node.
         * @param _B The second node.
         * @return The squared Euclidean distance between _A and _B.
         */
        [[nodiscard]] static constexpr auto SqrEuclideanDistance(const coord_t& _A, const coord_t& _B) {

            Ts result(0);

            for (size_t i = 0U; i < Kd; ++i) {
                const auto val = _B[i] - _A[i];
                result += val * val;
            }

            return result;
        }

        /**
          * @brief Calculate the Manhattan distance between two nodes.
          *
          * @tparam Kd The number of dimensions of the nodes.
          * @param _A The first node.
          * @param _B The second node.
          * @return The Manhattan distance between _A and _B.
          */
        [[nodiscard]] static constexpr auto ManhattanDistance(const coord_t& _A, const coord_t& _B) {

            Ts result(0);

            constexpr bool is_32bit((sizeof(size_t) * 8U) == 32U);
            constexpr bool is_64bit((sizeof(size_t) * 8U) == 64U);

            if constexpr (Kd > 0U) {

                if constexpr (Kd == 1U) {
                    result = _B[0U] - _A[0U];
                }

                //TODO: Add SIMD for AVX2 (and possibly AVX-512 too?).
#ifdef __SSE2__
                else if constexpr (is_64bit) {

                    if constexpr (Kd == 2U) {

#ifdef __GNUC__
                        _mm_prefetch((const char*)&_A[8], _MM_HINT_T0);
                        _mm_prefetch((const char*)&_B[8], _MM_HINT_T0);
#endif // __GNUC__

                        result = AStar::simd_sub_64bit_128(_A, _B);
                    }
                    else {

                        constexpr auto r = Kd % 2U;

                        size_t i;
                        for (i = 0U; i < (Kd - r); i += 2U) {

#ifdef __GNUC__
                            _mm_prefetch((const char*)&_A[8], _MM_HINT_T0);
                            _mm_prefetch((const char*)&_B[8], _MM_HINT_T0);
#endif // __GNUC__

                            result += AStar::simd_sub_64bit_128(
                                _A[i], _A[i + 1],
                                _B[i], _B[i + 1]
                            );
                        }

                        if constexpr (r != 0U) {
                            result += _B[i] - _A[i];
                        }
                    }
                }
                else if constexpr (is_32bit) {

                    if constexpr (Kd == 3U) {

#ifdef __GNUC__
                        _mm_prefetch((const char*)&_A[8], _MM_HINT_T0);
                        _mm_prefetch((const char*)&_B[8], _MM_HINT_T0);
#endif // __GNUC__

                        result = AStar::simd_sub_32_bit_128(
                            0U, _A[0], _A[1], _A[2],
                            0U, _B[0], _B[1], _B[2]
                        );
                    }
                    if constexpr (Kd == 4U) {
                        result = AStar::simd_sub_32_bit_128(_A, _B);
                    }
                    else {

                        constexpr auto r = Kd % 4U;

                        size_t i;
                        for (i = 0U; i < (Kd - r); i += 4U) {

#ifdef __GNUC__
                            _mm_prefetch((const char*)&_A[8], _MM_HINT_T0);
                            _mm_prefetch((const char*)&_B[8], _MM_HINT_T0);
#endif // __GNUC__

                            result += AStar::simd_sub_32bit_128(
                                _A[i], _A[i + 1], _A[i + 2], _A[i + 3],
                                _B[i], _B[i + 1], _B[i + 2], _B[i + 3]
                            );
                        }

                        for (; i < Kd; ++i) {
                            result += _B[i] - _A[i];
                        }
                    }
                }
#endif // __SSE2__
                else {

                    for (size_t i = 0U; i < Kd; ++i) {
                        result += _B[i] - _A[i];
                    }
                }

                return result;
            }
        }

#pragma endregion Heuristics

    };
} // CHDR::Solvers

#endif //CHDR_ASTAR_HPP