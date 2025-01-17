/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_SOLVER_HPP
#define CHDR_SOLVER_HPP

#include <cstddef>
#include <type_traits>
#include <vector>

#include "../../types/containers/existence_set.hpp"

// ReSharper disable once CppUnusedIncludeDirective
#include "../../utils/intrinsics.hpp" // NOLINT(*-include-cleaner)

namespace chdr::solvers {

    template <template <typename params_t> typename Derived, typename params_t>
    class solver final {

    public:

        template <typename T>
        struct is_graph : std::is_same<std::decay_t<T>, mazes::graph<typename params_t::index_type, typename params_t::scalar_type>>{};

        struct node_data {

            const bool active;

            const typename params_t:: coord_type coord;
            const typename params_t:: index_type index;
            const typename params_t::scalar_type distance;
        };

        template <typename maze_neighbour_t>
        static constexpr node_data get_data(maze_neighbour_t& _n, const params_t& _params) noexcept {

            if constexpr (is_graph<decltype(_params.maze)>::value) {

                // _params.maze is a graph...
                const auto& [nIndex, nDistance] = std::move(_n);

                constexpr bool nActive = true;
                auto nCoord = utils::to_nd(nIndex, _params.size);

                return {
                    nActive,
                    nCoord,
                    nIndex,
                    nDistance
                };
            }
            else {

                // _params.maze is a grid...
                const auto& [nActive, nCoord] = std::move(_n);

                const auto nIndex = static_cast<typename params_t::index_type>(
                    nActive ? utils::to_1d(nCoord, _params.size) : typename params_t::index_type{}
                );

                constexpr auto nDistance = static_cast<typename params_t::scalar_type>(1);

                return {
                    nActive,
                    nCoord,
                    nIndex,
                    nDistance
                };
            }
        }

        static constexpr size_t determine_capacity(const params_t& _params) noexcept {

            if constexpr (is_graph<decltype(_params.maze)>::value) {

                return static_cast<size_t>(
                    _params.capacity != 0U ?
                        _params.capacity :
                        utils::max(_params.maze.count() / 10U, static_cast<size_t>(1U))
                );
            }
            else {
                return utils::max(
                    _params.capacity,
                    utils::max(
                        static_cast<size_t>(utils::to_1d(_params.start, _params.size)),
                        static_cast<size_t>(utils::to_1d(_params.end,   _params.size))
                    )
                );
            }
        }

        [[maybe_unused, nodiscard]] constexpr auto operator()() const {
            return operator()(params_t {});
        }

        template <typename... Args>
        [[maybe_unused, nodiscard]]
#if __cplusplus >= 202003L
        constexpr
#endif // __cplusplus >= 202003L
        auto operator()(Args&&... _args) const {
            return operator()({ std::forward<Args>(_args)... });
        }

        [[maybe_unused, nodiscard]]
#if __cplusplus >= 202003L
        constexpr
#endif // __cplusplus >= 202003L
        auto operator()(const params_t& _params) const {

            using solver_t = Derived<params_t>;

            const auto s = utils::to_1d(_params.start, _params.size);
            const auto e = utils::to_1d(_params.end,   _params.size);

            if (_params.maze.contains(s) && _params.maze.at(s).is_active() &&
                _params.maze.contains(e) && _params.maze.at(e).is_active()
            ) {
                auto result = s != e ? solver_t::execute(_params) : std::vector<typename params_t::coord_type> { _params.end };
                
                if constexpr (std::is_invocable_v<decltype(&std::remove_reference_t<decltype(*_params.monotonic_pmr)>::release), decltype(*_params.monotonic_pmr)>) {
                    _params.monotonic_pmr->release();
                }

                return result;
            }

            return std::vector<typename params_t::coord_type>{};
        }
    };

    struct solver_utils final {

        template <typename T, typename collection_t>
        static constexpr void preallocate_emplace(collection_t& _collection, const T& _value, const size_t& _increment, const size_t& _max_increment = std::numeric_limits<size_t>::max()) {

            if constexpr (std::is_same_v<collection_t, existence_set<>>) {
                _collection.allocate(_value, _increment, _max_increment);
            }

            _collection.emplace(_value);
        }

        template<typename node_t, typename coord_t>
        static constexpr auto rbacktrack(const node_t& _node, const coord_t& _size) {

            std::vector<coord_t> result;

            {
                size_t depth = 0U;
                for (const auto* RESTRICT t = &_node; t->m_parent != nullptr; t = static_cast<const node_t*>(t->m_parent), ++depth) {}

                result.resize(depth);
            }

            size_t i = 0U;
            for (const auto* RESTRICT t = &_node; t->m_parent != nullptr; t = static_cast<const node_t*>(t->m_parent), ++i) {
                result[(result.size() - 1U) - i] = utils::to_nd(t->m_index, _size);
            }

            return result;
        }

        template<typename node_t, typename coord_t>
        static constexpr auto rbacktrack(const node_t& _node, const coord_t& _size, const size_t& _depth) {

            std::vector<coord_t> result(_depth);

            size_t i = 0U;
            for (const auto* RESTRICT t = &_node; t->m_parent != nullptr; t = static_cast<const node_t*>(t->m_parent), ++i) {
                result[(result.size() - 1U) - i] = utils::to_nd(t->m_index, _size);
            }

            return result;
        }

        template <typename open_set_t, typename coord_t>
        [[nodiscard]] static constexpr auto ibacktrack(const open_set_t& _open, const coord_t& _size) {

            std::vector<coord_t> result;
            result.reserve(_open.size());

            for (auto it = _open.rbegin(); it != _open.rend(); ++it) {
                result.emplace_back(utils::to_nd(it->m_index, _size));
            }

            return result;
        }
    };

    template <template <typename params_t> typename Derived, typename params_t>
    [[nodiscard]] static
#if __cplusplus >= 2023L
    constexpr
#endif // __cplusplus >= 2023L
    auto make_solver() {
        return solver<Derived, params_t>();
    }

    template <template <typename params_t> typename Derived, typename params_t>
    [[nodiscard]] static
#if __cplusplus >= 2023L
    constexpr
#endif // __cplusplus >= 2023L
    auto solve() {
        return solver<Derived, params_t>()();
    }

    template <template <typename params_t> typename Derived, typename params_t, typename... Args>
    [[nodiscard]] static
#if __cplusplus >= 2023L
    constexpr
#endif // __cplusplus >= 2023L
    auto solve(Args&&... _args) {
        return solver<Derived, params_t>()(std::forward<Args>(_args)...);
    }

    template <template <typename params_t> typename Derived, typename params_t>
    [[nodiscard]] static
    #if __cplusplus >= 2023L
        constexpr
    #endif // __cplusplus >= 2023L
    auto solve(const params_t& _params) {
        return solver<Derived, params_t>()(_params);
    }

} //chdr::solvers

#endif //CHDR_SOLVER_HPP