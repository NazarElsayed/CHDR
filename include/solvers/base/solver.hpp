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
#include "../../utils/utils.hpp"

// ReSharper disable once CppUnusedIncludeDirective
#include "../../utils/intrinsics.hpp" // NOLINT(*-include-cleaner)

namespace chdr::solvers {

    /**
     * @nosubgrouping
     * @brief
     * @tparam Derived
     * @tparam params_t
     */
    template <template <typename params_t> typename Derived, typename params_t>
    class solver final {

        friend class Derived<params_t>;

    private:

         struct solver_utils final {

                 solver_utils()                                = delete;
                 solver_utils           (const solver_utils& ) = delete;
                 solver_utils           (const solver_utils&&) = delete;
                 solver_utils& operator=(const solver_utils& ) = delete;
                 solver_utils& operator=(const solver_utils&&) = delete;
                ~solver_utils()                                = delete;

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

                template <typename T, typename collection_t>
                HOT static constexpr void preallocate_emplace(collection_t& _collection, const T& _value, size_t _increment, size_t _max_increment = std::numeric_limits<size_t>::max()) {

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
                static constexpr auto rbacktrack(const node_t& _node, const coord_t& _size, size_t _depth) {

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

         /**
          * @struct is_graph
          * @brief A utility trait to check if a type is a graph type.
          *
          * @details This struct determines if a given type `T`, when decayed, is equivalent to the graph
          *          type `mazes::graph` with index and scalar types supplied by parameters from `params_t`.
          *          This comparison utilises `std::is_same` and `std::decay_t` for precise type matching.
          *
          * @note This trait is primarily used within the `solver` class to ensure compatibility with graph
          *       structures in `mazes`, specifically verifying if the type satisfies graph type requirements
          *       for operations.
          *
          * @tparam T The type to be checked against the graph type.
          */
         template <typename T>
         struct is_graph : std::is_same<std::decay_t<T>, mazes::graph<typename params_t::index_type, typename params_t::scalar_type>>{};

         /**
          * @struct resettable
          * @brief A utility trait for determining whether a type has a callable `reset` method.
          *
          * @details This struct uses SFINAE (Substitution Failure Is Not An Error) to identify if a given type
          *          contains a publicly accessible member function named `reset`. The value of the trait will be
          *          `true` if the `reset` method exists, and `false` otherwise.
          *
          * @tparam T The type to be inspected for the presence of a `reset` method.
          */
         template <typename T>
         struct resettable final {

     private:

         template <typename U>
         static constexpr auto has_method(int) -> decltype(&U::reset, std::true_type{});

         template <typename>
         static constexpr std::false_type has_method(...);

     public:

        /**
         * @brief Determines the value of the `resettable` trait for a given type.
         *
         * @details This constant expression evaluates to `true` if the specified type `T`
         *          possesses a publicly accessible `reset` method. The implementation
         *          leverages a combination of SFINAE (Substitution Failure Is Not An Error)
         *          and the detection idiom to ascertain the existence of the method.
         *
         * @note This is a static constexpr boolean value used at compile time for type traits
         *       evaluation.
         *
         * @tparam T The type being checked for the presence of a callable `reset` method.
         */
         static constexpr bool value = decltype(has_method<T>(0))::value;

     };

         /**
          * @struct node_data
          * @brief Represents data associated with a node from a search space in pathfinding algorithms.
          *
          * @details This structure encapsulates essential information about a node, including its
          *          activity state, index, coordinates, and distance. It is primarily utilised in search
          *          algorithms such as A* and its variants to store and process information related to
          *          graph traversal.
          *
          * @note All member fields of this structure are immutable after initialisation.
          */
         struct node_data {

             /** @brief Indicates whether the node is active and should be considered in the computation logic. */
             const bool active;

             /** @brief The index of the node, typically representing its unique identifier within the structure of the search space. */
             const typename params_t:: index_type index;

             /** @brief The coordinates of the node in the search space, relevant for spatial algorithms. */
             const typename params_t:: coord_type coord;

             /** @brief The computed distance metric (e.g., cost or heuristic) associated with this node relative to its neighbours. */
             const typename params_t::scalar_type distance;
         };

         /**
          * @brief Retrieves node data based on the maze type and neighbour information.
          *
          * @details This method constructs and returns a `node_data` object by interpreting
          *          the provided neighbour information (`_n`) and parameters (`_params`). It
          *          adapts its behaviour depending on whether the maze is a graph or a grid:
          *
          *          - For a graph: Extracts and uses the node index and distance.
          *          - For a grid: Derives the node index from coordinates and sets the distance
          *            as a constant scalar value.
          *
          * @param [in] _n A reference to the maze neighbour structure, which contains the
          *                information necessary for constructing the node data. For graphs,
          *                this includes the index and distance. For grids, this includes the
          *                activity state and coordinates.
          *
          * @param [in] _params A constant reference to the parameters object of type `params_t`,
          *                     which contains metadata (e.g., maze dimensions) required for
          *                     processing the neighbour information.
          *
          * @tparam maze_neighbour_t The type representing the maze neighbour.
          *
          * @return A `node_data` object with fields populated based on the maze type and the
          *         input parameters.
          *
          * @see node_data
          */
         template <typename maze_neighbour_t>
         static constexpr node_data get_data(const maze_neighbour_t& _n, const params_t& _params) noexcept {

             if constexpr (is_graph<decltype(_params.maze)>::value) {

                 // _params.maze is a graph...
                 const auto& [nIndex, nDistance](_n);

                 return {
                     true,
                     nIndex,
                     utils::to_nd(nIndex, _params.size),
                     nDistance
                 };
             }
             else {

                 // _params.maze is a grid...
                 const auto& [nActive, nCoord](_n);

                 const auto nIndex = static_cast<typename params_t::index_type>(
                     nActive ? utils::to_1d(nCoord, _params.size) : typename params_t::index_type{}
                 );

                 constexpr auto nDistance = static_cast<typename params_t::scalar_type>(1);

                 return {
                     nActive,
                     nIndex,
                     nCoord,
                     nDistance
                 };
             }
         }

         /**
          * @name Constructors
          * @{
          */

         solver           (const solver& ) = delete;
         solver           (const solver&&) = delete;
         solver& operator=(const solver& ) = delete;
         solver& operator=(const solver&&) = delete;

         /**
          * @}
          */

    public:

        /**
         * @name Constructors
         * @{
         */

        /**
         * @brief Constructor for the `solver` class.
         *
         * @details Initialises a `solver` instance with default values.
         *
         * @return Instance of the `solver` class.
         */
        [[nodiscard]] constexpr solver() noexcept = default;
        ~solver() = default;

        /**
         * @}
         */

        /**
         * @brief Executes the solver with the provided parameters. Parameters are constructed using perfect forwarding.
         *
         * @details Constructs a parameters object using the given arguments, and uses it to execute the desired solver.
         *
         * @note You must ensure that the given arguments are valid for constructing the parameters for your intended search.
         *
         * @tparam Args A parameter pack containing the arguments to construct the parameters object.
         * @return A std::vector containing the result of the search. If the search fails, the vector will be empty.
         */
        template <typename... Args>
        [[maybe_unused, nodiscard]]
        static
#if __cplusplus >= 202003L
        constexpr
#endif // __cplusplus >= 202003L
        auto solve(Args&&... _args) {
            return operator()(std::forward<Args>(_args)...);
        }

        /**
         * @brief Executes the solver with the provided parameters.
         *
         * @param _params The parameters object to solve with.
         *
         * @return A std::vector containing the result of the search. If the search fails, the vector will be empty.
         */
        [[maybe_unused, nodiscard]]
        static
#if __cplusplus >= 202003L
        constexpr
#endif // __cplusplus >= 202003L
        auto solve(const params_t& _params) {
            return operator()(_params);
        }

        /**
         * @brief Executes the solver with the provided parameters. Parameters are constructed using perfect forwarding.
         *
         * @details Constructs a parameters object using the given arguments, and uses it to execute the desired solver.
         *
         * @note You must ensure that the given arguments are valid for constructing the parameters for your intended search.
         *
         * @tparam Args A parameter pack containing the arguments to construct the parameters object.
         * @return A std::vector containing the result of the search. If the search fails, the vector will be empty.
         */
        template <typename... Args>
        [[maybe_unused, nodiscard]]
        static
#if __cplusplus >= 202003L
        constexpr
#endif // __cplusplus >= 202003L
        auto operator()(Args&&... _args) {
            return operator()({ std::forward<Args>(_args)... });
        }

        /**
         * @brief Executes the solver with the provided parameters.
         *
         * @param _params The parameters object to solve with.
         *
         * @return A std::vector containing the result of the search. If the search fails, the vector will be empty.
         */
        [[maybe_unused, nodiscard]]
        static
#if __cplusplus >= 202003L
        constexpr
#endif // __cplusplus >= 202003L
        auto operator()(const params_t& _params) {

            using solver_t = Derived<params_t>;

            const auto s = static_cast<typename params_t::index_type>(utils::to_1d(_params.start, _params.size));
            const auto e = static_cast<typename params_t::index_type>(utils::to_1d(_params.end,   _params.size));

            if (_params.maze.contains(s) && _params.maze.at(s).is_active() &&
                _params.maze.contains(e) && _params.maze.at(e).is_active()
            ) {
                auto result = s != e ? solver_t::execute(_params) : std::vector<typename params_t::coord_type> { _params.end };

                if constexpr (resettable<std::remove_reference_t<decltype(*_params.monotonic_pmr)>>::value) {
                    if (_params.monotonic_pmr != nullptr) { _params.monotonic_pmr->reset();}
                }
                if constexpr (resettable<std::remove_reference_t<decltype(*_params.polytonic_pmr)>>::value) {
                    if (_params.polytonic_pmr != nullptr) { _params.polytonic_pmr->reset(); }
                }
                if constexpr (resettable<std::remove_reference_t<decltype(*_params.pool_pmr)>>::value) {
                    if (_params.pool_pmr      != nullptr) { _params.pool_pmr->reset(); }
                }

                return result;
            }

            return std::vector<typename params_t::coord_type>{};
        }
    };

} //chdr::solvers

#endif //CHDR_SOLVER_HPP