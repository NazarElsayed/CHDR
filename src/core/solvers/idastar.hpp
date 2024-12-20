/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_IDASTAR_HPP
#define CHDR_IDASTAR_HPP

#include <cstddef>
#include <limits>
#include <vector>

#include "base/solver.hpp"
#include "types/coord.hpp"
#include "utils/intrinsics.hpp"
#include "utils/utils.hpp"

namespace chdr::solvers {

    template<size_t Kd, typename scalar_t, typename index_t, typename params_t>
    struct [[maybe_unused]] idastar final {

        friend struct solver<idastar, Kd, scalar_t, index_t, params_t>;

        static_assert(std::is_integral_v<scalar_t> || std::is_floating_point_v<scalar_t>, "scalar_t must be either an integral or floating point type.");
        static_assert(std::is_integral_v<index_t>, "index_t must be an integral type.");

    private:

        using solver_t = solver<idastar, Kd, scalar_t, index_t, params_t>;
        using  coord_t = coord<index_t, Kd>;

        struct node final : bnode<index_t> {

            scalar_t m_gScore;
            scalar_t m_fScore;

            /**
             * @brief Constructs an uninitialized ASNode.
             *
             * This constructor creates an ASNode with uninitialized members.
             */
            [[nodiscard]] constexpr node() noexcept : bnode<index_t>() {} // NOLINT(*-pro-type-member-init, *-use-equals-default)

            [[nodiscard]] constexpr node(const index_t& _index, const scalar_t& _gScore, const scalar_t& _fScore) noexcept : bnode<index_t>(_index),
                m_gScore(_gScore),
                m_fScore(_fScore) {}

            [[nodiscard]] friend constexpr bool operator < (const node& _a, const node& _b) noexcept {
                return _a.m_fScore == _b.m_fScore ?
                       _a.m_gScore >  _b.m_gScore :
                       _a.m_fScore >  _b.m_fScore;
            }
        };

        template <typename open_set_t>
        static constexpr scalar_t search(open_set_t& _open, const scalar_t& _bound, const params_t& _params) {

            static_assert(std::numeric_limits<scalar_t>::is_specialized, "scalar_t must be a numeric type with defined numeric limits.");

            const auto e = utils::to_1d(_params.end, _params.size);

            auto min = std::numeric_limits<scalar_t>::max();

            const auto& curr = _open.back();

            if (curr.m_fScore <= _bound) {

                for (const auto& n_data : _params.maze.get_neighbours(curr.m_index)) {

                    if (const auto& n = solver_t::get_data(n_data, _params); n.active) {

                        if (!std::any_of(_open.begin(), _open.end(), [&n](const auto& _item) ALWAYS_INLINE { return _item.m_index == n.index; })) {

                            _open.emplace_back(n.index, curr.m_gScore + n.distance, _params.h(n.coord, _params.end) * _params.weight);

                            if (n.index != e) {
                                min = std::min(min, search(_open, _bound, _params));
                                _open.pop_back();
                            }
                            else {
                                min = _bound;
                                break;
                            }
                        }
                    }
                }
            }

            return min;
        }

        template <typename open_set_t>
        static constexpr auto solve_internal(open_set_t& _open, const params_t& _params) {

            const auto s = utils::to_1d(_params.start, _params.size);
            const auto e = utils::to_1d(_params.end,   _params.size);

            auto bound = _params.h(_params.start, _params.end) * _params.weight;

            _open.emplace_back(s, static_cast<scalar_t>(0), bound);

            // Main loop:
            while (!_open.empty()) {

                const auto& curr = _open.back();

                if (curr.m_index != e) { // SEARCH FOR SOLUTION...
                    bound = search(_open, bound, _params);
                }
                else { // SOLUTION REACHED ...

                    // Reserve space in result:
                    std::vector<coord_t> result;
                    result.reserve(curr.m_gScore);

                    // Recurse from end node to start node, inserting into a result buffer:
                    for (auto& t : _open) {
                        result.emplace_back(utils::to_nd(t.m_index, _params.size));
                    }

                    // Reverse the result:
                    std::reverse(result.begin(), result.end());

                    return result;
                }
            }

            return std::vector<coord_t>{};
        }

        [[maybe_unused, nodiscard]] static constexpr auto execute(const params_t& _params) {

            const auto capacity = solver_t::determine_capacity(_params);

            std::vector<node> open;
            open.reserve(capacity / 8U);

            return solve_internal(open, _params);
        }
    };

} //chdr::solvers

#endif //CHDR_IDASTAR_HPP