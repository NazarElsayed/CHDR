/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_SOLVER_HPP
#define CHDR_SOLVER_HPP

/**
 * @file solver.hpp
 */

#include <cstddef>
#include <limits>
#include <type_traits>
#include <vector>

#include "../../utils/utils.hpp"

// ReSharper disable once CppUnusedIncludeDirective
#include <iostream>

#include "../../utils/intrinsics.hpp" // NOLINT(*-include-cleaner)

namespace chdr::solvers {

    /**
     * @nosubgrouping
     * @class solver
     * @brief A static class serving as the core entry point for the CHDR library.
     * @details The `solver` class provides a unified interface for instantiating, configuring, and invoking various solvers.
     *          It consolidates common utilities and functionalities required to create and manage search or optimisation instances.
     *          This class is designed for flexibility and extensibility, supporting a wide range of solvers through the use of
     *          template specialisations.
     *          
     *          Key features include:
     *          - Integration with customisable parameter sets defined by `params_t`.
     *          - Specialized utility traits to enforce type and method requirements during compilation.
     *          - Support for user-defined solver implementations via the `solver_t` template parameter.
     *
     * @code
     *
     * // Example usage. Replace [...] with your values.
     * struct params {
     *
     *     using  weight_type [[maybe_unused]] = ...; // char
     *     using  scalar_type [[maybe_unused]] = ...; // uint32_t
     *     using   index_type [[maybe_unused]] = ...; // uint32_t
     *     using   coord_type [[maybe_unused]] = ...; // chdr::coord<uint32_t, 2>
     *
     *     using        lazy_sorting [[maybe_unused]] = ...; // std::false_type / std::true_type;
     *     using          no_cleanup [[maybe_unused]] = ...; // std::false_type / std::true_type;
     *     using reverse_equivalence [[maybe_unused]] = ...; // std::false_type / std::true_type;
     *
     *     const        ... maze;  // i.e. chdr::grid, chdr::graph
     *     const coord_type start;
     *     const coord_type end;
     *     const coord_type size;
     *          scalar_type (*h)(const coord_type&, const coord_type&) noexcept;
     *
     *     ...*     monotonic_pmr;
     *     ...* heterogeneous_pmr;
     *     ...*   homogeneous_pmr;
     *
     *     const scalar_type weight       = ...; // 1
     *     const      size_t capacity     = ...; // 0
     *     const      size_t memoryLimit  = ...; // MAX
     * };
     *
     * // Invoke with chosen algorithm and parameters. Here, A* is used:
     * auto path = chdr::solvers::solver<chdr::solvers::astar, params>::solve(
     *     my_maze,
     *     my_start,
     *     my_end,
     *     my_size,
     *     chdr::heuristics::manhattan_distance<params::scalar_t, params::coord_t>,
     *     my_monotonic_pmr,
     *     my_polytonic_pmr,
     *     my_homogeneous_pmr
     * );
     * @endcode
     *
     * @tparam solver_t The solver algorithm to be used, which must adhere to the interface defined by the `solver` class.
     * @tparam params_t A parameter type that defines the configuration specifics for the solver, including graph, spatial, and other constraints.
     */
    template <template <typename params_t> typename solver_t, typename params_t>
    class solver final {

        friend struct solver_t<params_t>;

    private:

        struct solver_utils final {

            friend solver;
            friend struct solver_t<params_t>;

            solver_utils()                                = delete;
            solver_utils           (const solver_utils& ) = delete;
            solver_utils           (const solver_utils&&) = delete;
            solver_utils& operator=(const solver_utils& ) = delete;
            solver_utils& operator=(const solver_utils&&) = delete;
            ~solver_utils()                               = delete;

        private:

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
             * @brief Determines whether the specified type is a `chdr::mazes::graph`.
             *
             * @details This compile-time constant evaluates to `true` if the type `T`
             *          is a `chdr::mazes::graph` matching the expected signature.
             *          Utilises a type trait to perform the detection via
             *          metaprogramming techniques.
             *
             * @note This value is commonly used in template specialisations or static
             *       assertions to enable or restrict functionality depending on the
             *       given type `T`.
             *
             * @tparam T The type being evaluated.
             *
             * @return true, if the given type is a `chdr::mazes::graph`. Otherwise, false.
             */
            template <typename T>
            static constexpr bool is_graph_v = is_graph<T>::value;

            /**
             * @struct has_method_reset
             * @brief A utility trait for determining whether a type has a callable `reset` method.
             *
             * @details This struct uses SFINAE (Substitution Failure Is Not An Error) to identify if a given type
             *          contains a publicly accessible member function named `reset`. The value of the trait will be
             *          `true` if the `reset` method exists, and `false` otherwise.
             *
             * @tparam T The type to be inspected for the presence of a `reset` method.
             */
            template <typename T>
            struct has_method_reset final {

             private:

                 template <typename U>
                 static constexpr decltype(&U::reset, std::true_type{}) has_method([[maybe_unused]] int _i) {
                     static_assert(std::true_type::value, "This function should not be called at run time.");
                     return {};
                 }

                 template <typename>
                 static constexpr std::false_type has_method(...) {
                     static_assert(std::true_type::value, "This function should not be called at run time.");
                     return {};
                 }

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
                 static constexpr bool value = decltype(has_method<std::remove_reference_t<T>>(0))::value;

            };

            /**
             * @brief Determines whether the specified type possesses a `reset` method.
             *
             * @details This compile-time constant evaluates to `true` if the type `T`
             *          contains a publicly accessible `reset` method that matches
             *          the expected signature. Utilises a type trait to perform the
             *          detection through the use of template metaprogramming techniques.
             *
             * @note This value is commonly used in template specialisations or static
             *       assertions to enable or restrict functionality depending on the
             *       presence of the method in the given type `T`.
             *
             * @tparam T The type being evaluated for the existence of a callable `reset` method.
             *
             * @return true, if the given type has the method. Otherwise, false.
             */
            template <typename T>
            static constexpr bool has_method_reset_v = has_method_reset<T>::value;

            /**
             * @struct has_method_reset
             * @brief A utility trait for determining whether a type has a callable `allocate` method.
             *
             * @details This struct uses SFINAE (Substitution Failure Is Not An Error) to identify if a given type
             *          contains a publicly accessible member function named `allocate`. The value of the trait will be
             *          `true` if the `allocate` method exists, and `false` otherwise.
             *
             * @tparam T The type to be inspected for the presence of a `allocate` method.
             */
            template <typename T>
            struct has_method_allocate final {

            private:

                template <typename U>
                static constexpr decltype(&U::reset, std::true_type{}) has_method([[maybe_unused]] int _i) {
                    static_assert(std::true_type::value, "This function should not be called at run time.");
                    return {};
                }

                template <typename>
                static constexpr std::false_type has_method(...) {
                    static_assert(std::true_type::value, "This function should not be called at run time.");
                    return {};
                }

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
             * @brief Determines whether the specified type possesses an `allocate` method.
             *
             * @details This compile-time constant evaluates to `true` if the type `T`
             *          contains a publicly accessible `allocate` method that matches
             *          the expected signature. Utilises a type trait to perform the
             *          detection through the use of template metaprogramming techniques.
             *
             * @note This value is commonly used in template specialisations or static
             *       assertions to enable or restrict functionality depending on the
             *       presence of the method in the given type `T`.
             *
             * @tparam T The type being evaluated for the existence of a callable `allocate` method.
             *
             * @return true, if the given type has the method. Otherwise, false.
             */
            template <typename T>
            static constexpr bool has_method_allocate_v = has_method_allocate<T>::value;

            /**
             * @struct has_method_allocate
             * @brief A utility trait for determining whether a type has a callable `allocate` method.
             *
             * @details This struct uses SFINAE (Substitution Failure Is Not An Error) to identify if a given type
             *          contains a publicly accessible member function named `allocate`. The value of the trait will be
             *          `true` if the `allocate` method exists, and `false` otherwise.
             *
             *          The trait is commonly used in template metaprogramming to conditionally enable functionality
             *          based on whether a type provides allocation capabilities.
             *
             * @tparam T The type to be inspected for the presence of an `allocate` method.
             */
            template <typename T>
            struct has_subscript_operator final {

            private:

                template <typename U>
                static constexpr decltype(std::declval<U>()[0], std::true_type{}) has_operator([[maybe_unused]] int _i) {
                    static_assert(std::true_type::value, "This function should not be called at run time.");
                    return {};
                }

                template <typename>
                static constexpr std::false_type has_operator(...) {
                    static_assert(std::true_type::value, "This function should not be called at run time.");
                    return {};
                }

            public:

                /**
                 * @brief Determines whether the specified type has a subscript operator.
                 *
                 * @details This constant expression evaluates to `true` if the type `T`
                 *          contains a publicly accessible subscript operator that matches
                 *          the expected signature. The implementation leverages a combination
                 *          of SFINAE (Substitution Failure Is Not An Error) and the detection
                 *          idiom to ascertain the existence of the operator.
                 *
                 * @note This is a static constexpr boolean value used at compile time for type traits
                 *       evaluation.
                 *
                 * @tparam T The type being checked for the presence of a callable subscript operator.
                 */
                static constexpr bool value = decltype(has_operator<T>(0))::value;
            };

            /**
             * @brief Determines whether the specified type possesses a subscript operator.
             *
             * @details This compile-time constant evaluates to `true` if the type `T`
             *          contains an accessible subscript operator.
             *          Utilises a type trait to perform the
             *          detection through the use of template metaprogramming techniques.
             *
             * @note This value is commonly used in template specialisations or static
             *       assertions to enable or restrict functionality depending on the
             *       presence of the operator in the given type `T`.
             *
             * @tparam T The type being evaluated for the existence of a subscript operator.
             *
             * @return true, if the given type has the operator. Otherwise, false.
             */
            template <typename T>
            static constexpr bool has_subscript_operator_v = has_subscript_operator<T>::value;

            /**
             * @brief Estimates the preallocation quota for a given solve.
             *
             * @details This method estimates the required container capacity needed during maze-solving
             *          operations. If the capacity parameter is explicitly provided, it used directly.
             *          Otherwise, the calcuation estimates a preallocation quota based on the size of
             *          the search space.
             *
             * @param [in] _params An object containing parameters for the maze-solving process.
             *
             * @warning For sufficiently large mazes, the capacity value produce may exceed available system memory.
             *          It may therefore be desirable to attempt presizing inside a `try, catch` block, and continue
             *          without presizing in the event of an allocation failure.
             *
             * @return The computed capacity as a `size_t` value.
             */
            static constexpr size_t determine_capacity(const params_t& _params) noexcept {

                if constexpr (is_graph_v<decltype(_params.maze)>) {

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

            /**
             * @brief Preallocates resources and inserts a value into the specified collection.
             *
             * @details This method preallocates memory for the given collection type if it supports a method called `allocate`.
             *          Following preallocation, the provided value is emplaced into the collection.
             *
             * @param [in, out] _collection The collection where the allocation and emplaced value are applied.
             *
             * @param [in] _value The value to be inserted or emplaced within the collection.
             * @param _increment The incremental size for preallocation, relevant for `existence_set`.
             * @param _max_increment (optional) The maximum allowable increment for preallocation. By default,
             *                                  it is set to the largest value of `size_t`.
             *
             * @note This method utilises `std::is_same_v` to determine if the collection is of type `existence_set`.
             *       If it is not, no preallocation occurs, and the value is directly emplaced into the collection.
             *
             * @tparam T The type of the value to be inserted into the collection.
             * @tparam collection_t The type of the collection into which values are to be emplaced.
             */
            template <typename T, typename collection_t>
            HOT static constexpr void preallocate_emplace(collection_t& _collection, const T& _value, size_t _increment, size_t _max_increment = std::numeric_limits<size_t>::max()) {

                if constexpr (has_method_allocate_v<collection_t>) {
                    _collection.allocate(_value, _increment, _max_increment);
                }

                _collection.emplace(_value);
            }

            /**
             * @brief Resets allocated resources associated with the solver parameters.
             *
             * @details This method attempts to reset resources held by the polymorphic memory resources
             *          specified in the parameters. It handles three types of resources:
             *          - monotonic_pmr
             *          - heterogeneous_pmr
             *          - homogeneous_pmr
             *
             *          The method implements deferred exception handling, capturing the first exception
             *          that occurs during resource release while attempting to release all resources.
             *
             * @param [in] _params The parameter object containing the resources to reset.
             *
             * @throws Rethrows any exception that occurred during resource release.
             *
             * @warning The method expects the memory resources to have a visible release() method.
             *          Aliasing the resources as other types may hide this method and prevent proper cleanup.
             */
            static constexpr void reset_resources(const params_t& _params) {

                std::exception_ptr e { nullptr };

                /*
                 * Reset resources with deferred exception handling:
                 */

                try {
                    if constexpr (solver_utils::template has_method_reset_v<std::remove_pointer_t<std::decay_t<decltype(_params.monotonic_pmr)>>>) {
                        if (_params.monotonic_pmr != nullptr) { _params.monotonic_pmr->reset(); }
                    }
                }
                catch (...) { // NOLINT(*-empty-catch)
                    if (e == nullptr) { e = std::current_exception(); }
                }

                try {
                    if constexpr (solver_utils::template has_method_reset_v<std::remove_pointer_t<std::decay_t<decltype(_params.heterogeneous_pmr)>>>) {
                        if (_params.heterogeneous_pmr != nullptr) { _params.heterogeneous_pmr->reset(); }
                    }
                }
                catch (...) { // NOLINT(*-empty-catch)
                    if (e == nullptr) { e = std::current_exception(); }
                }

                try {
                    if constexpr (solver_utils::template has_method_reset_v<std::remove_pointer_t<std::decay_t<decltype(_params.homogeneous_pmr)>>>) {
                        if (_params.homogeneous_pmr != nullptr) { _params.homogeneous_pmr->reset(); }
                    }
                }
                catch (...) { // NOLINT(*-empty-catch)
                    if (e == nullptr) { e = std::current_exception(); }
                }

                // Report first exception to occur:
                if (e != nullptr) {
                    std::rethrow_exception(e);
                }
            }

            /**
             * @brief Releases allocated resources associated with the solver parameters.
             *
             * @details This method attempts to release resources held by the polymorphic memory resources
             *          specified in the parameters. It handles three types of resources:
             *          - monotonic_pmr
             *          - heterogeneous_pmr
             *          - homogeneous_pmr
             *
             *          The method implements deferred exception handling, capturing the first exception
             *          that occurs during resource release while attempting to release all resources.
             *
             * @param [in] _params The parameter object containing the resources to be released.
             *
             * @throws Rethrows any exception that occurred during resource release.
             *
             * @warning The method expects the memory resources to have a visible release() method.
             *          Aliasing the resources as other types may hide this method and prevent proper cleanup.
             */
            static constexpr void release_resources(const params_t& _params) {

                std::exception_ptr e { nullptr };

                /*
                 * Reset resources with deferred exception handling:
                 */

                try {
                    if constexpr (solver_utils::template has_method_reset_v<std::remove_pointer_t<std::decay_t<decltype(_params.monotonic_pmr)>>>) {
                        if (_params.monotonic_pmr != nullptr) { _params.monotonic_pmr->release(); }
                    }
                }
                catch (...) { // NOLINT(*-empty-catch)
                    if (e == nullptr) { e = std::current_exception(); }
                }

                try {
                    if constexpr (solver_utils::template has_method_reset_v<std::remove_pointer_t<std::decay_t<decltype(_params.heterogeneous_pmr)>>>) {
                        if (_params.heterogeneous_pmr != nullptr) { _params.heterogeneous_pmr->release(); }
                    }
                }
                catch (...) { // NOLINT(*-empty-catch)
                    if (e == nullptr) { e = std::current_exception(); }
                }

                try {
                    if constexpr (solver_utils::template has_method_reset_v<std::remove_pointer_t<std::decay_t<decltype(_params.homogeneous_pmr)>>>) {
                        if (_params.homogeneous_pmr != nullptr) { _params.homogeneous_pmr->release(); }
                    }
                }
                catch (...) { // NOLINT(*-empty-catch)
                    if (e == nullptr) { e = std::current_exception(); }
                }

                // Report first exception to occur:
                if (e != nullptr) {
                    std::rethrow_exception(e);
                }
            }

            /**
             * @brief Constructs a reverse path from a given node to its root in a tree or graph structure.
             *
             * @details This function iteratively backtracks from the specified node `_node` to its root
             *          through its parent pointers, converting each node's index into a coordinate
             *          using the dimensions provided in `_size`. The resulting path is stored in reverse order,
             *          where the first element corresponds to the root node and the last element corresponds
             *          to the provided `_node`. The depth of the path is dynamically determined during the
             *          traversal process.
             *
             * @param [in] _node The node to begin backtracking from. This is assumed to be part of a structure
             *                   where nodes possess a parent pointer (`m_parent`) for traversal.
             * @param [in] _size The coordinate dimensions, used to convert an index to n-dimensional coordinates.
             *
             * @return A vector of n-dimensional coordinates representing the path from the root node
             *         to the provided `_node`.
             */
            template<typename node_t, typename coord_t>
            static constexpr auto rbacktrack(const node_t& _node, const coord_t& _size) {

                std::vector<coord_t> result{};

                {
                    size_t depth = 0U;
                    for (const auto* RESTRICT t = &_node; t->m_parent != nullptr; t = static_cast<const node_t*>(t->m_parent)) {
                        ++depth;
                    }

                    result.resize(depth);
                }

                size_t i = 0U;
                for (const auto* RESTRICT t = &_node; t->m_parent != nullptr; t = static_cast<const node_t*>(t->m_parent)) {
                    result[(result.size() - 1U) - i] = utils::to_nd(t->m_index, _size);
                    ++i;
                }

                return result;
            }

            /**
             * @brief Constructs a path from a given node to its root in a tree or graph structure.
             *
             * @details This function iteratively backtracks from the specified node `_node` to its root
             *          through its parent pointers, converting each node's index into a coordinate
             *          using the dimensions provided in `_size`.
             *          The depth of the path is dynamically determined during the traversal process.
             *
             * @param [in] _node The node to begin backtracking from. This is assumed to be part of a structure
             *                   where nodes possess a parent pointer (`m_parent`) for traversal.
             * @param [in] _size The coordinate dimensions, used to convert an index to n-dimensional coordinates.
             *
             * @return A vector of n-dimensional coordinates representing the path from the root node
             *         to the provided `_node`.
             */
            template<typename node_t, typename coord_t>
            static constexpr auto rbacktrack_noreverse(const node_t& _node, const coord_t& _size) {

                std::vector<coord_t> result{};

                for (const auto* RESTRICT t = &_node; t->m_parent != nullptr; t = static_cast<const node_t*>(t->m_parent)) {
                    result.emplace_back(utils::to_nd(t->m_index, _size));
                }

                return result;
            }

            /**
             * @brief Constructs a reverse path from a given node to its root in a tree or graph structure.
             *
             * @details This function iteratively backtracks from the specified node `_node` to its root
             *          through its parent pointers, converting each node's index into a coordinate
             *          using the dimensions provided in `_size`. The resulting path is stored in reverse order,
             *          where the first element corresponds to the root node and the last element corresponds
             *          to the provided `_node`. The depth of the path is static, provided using the `_depth` parameter.
             *
             * @param [in] _node The node to begin backtracking from. This is assumed to be part of a structure
             *                   where nodes possess a parent pointer (`m_parent`) for traversal.
             * @param [in] _size The coordinate dimensions, used to convert an index to n-dimensional coordinates.
             *
             * @param _depth The depth of the hierarchy, which determines the length of the resulting vector.
             *
             * @return A vector of n-dimensional coordinates representing the path from the root node
             *         to the provided `_node`.
             */
            template<typename node_t, typename coord_t>
            static constexpr auto rbacktrack(const node_t& _node, const coord_t& _size, size_t _depth) {

                std::vector<coord_t> result(_depth);

                size_t i = 0U;
                for (const auto* RESTRICT t = &_node; t->m_parent != nullptr; t = static_cast<const node_t*>(t->m_parent)) {
                    result[(result.size() - 1U) - i] = utils::to_nd(t->m_index, _size);
                    ++i;
                }

                return result;
            }

            /**
             * @brief Constructs a path from a given node to its root in a tree or graph structure.
             *
             * @details This function iteratively backtracks from the specified node `_node` to its root
             *          through its parent pointers, converting each node's index into a coordinate
             *          using the dimensions provided in `_size`.
             *          The depth of the path is static, provided using the `_depth` parameter.
             *
             * @param [in] _node The node to begin backtracking from. This is assumed to be part of a structure
             *                   where nodes possess a parent pointer (`m_parent`) for traversal.
             * @param [in] _size The coordinate dimensions, used to convert an index to n-dimensional coordinates.
            *
             * @param _depth The depth of the hierarchy, which determines the length of the resulting vector.
             *
             * @return A vector of n-dimensional coordinates representing the path from the root node
             *         to the provided `_node`.
             */
            template<typename node_t, typename coord_t>
            static constexpr auto rbacktrack_noreverse(const node_t& _node, const coord_t& _size, size_t _depth) {

                std::vector<coord_t> result(_depth);

                for (const auto* RESTRICT t = &_node; t->m_parent != nullptr; t = static_cast<const node_t*>(t->m_parent)) {
                    result.emplace_back(utils::to_nd(t->m_index, _size));
                }

                return result;
            }

            /**
             * @brief Constructs a reversed sequence of coordinates by backtracking through the open set.
             *
             * @details This method processes the given open set in reverse order, transforming each index
             *          into a coordinate based on the provided size parameter.
             *
             * @param [in] _open The open set container representing the path.
             * @param [in] _size The coordinate dimensions, used to convert an index to n-dimensional coordinates.
            *
             * @return A vector of n-dimensional coordinates representing the path from the root node
             *         to the provided `_node`.
             */
            template <typename open_set_t, typename coord_t>
            [[nodiscard]] static constexpr auto ibacktrack(const open_set_t& _open, const coord_t& _size) {

                std::vector<coord_t> result{};
                result.reserve(_open.size());

                const auto end = _open.rend();
                for (auto it = _open.rbegin(); it != end; ++it) {
                    result.emplace_back(utils::to_nd(it->m_index, _size));
                }

                return result;
            }

            /**
             * @brief Constructs a sequence of coordinates by backtracking through the open set.
             *
             * @details This method processes the given open set in reverse order, transforming each index
             *          into a coordinate based on the provided size parameter.
             *
             * @param [in] _open The open set container representing the path.
             * @param [in] _size The coordinate dimensions, used to convert an index to n-dimensional coordinates.
            *
             * @return A vector of n-dimensional coordinates representing the path from the root node
             *         to the provided `_node`.
             */
            template <typename open_set_t, typename coord_t>
            [[nodiscard]] static constexpr auto ibacktrack_noreverse(const open_set_t& _open, const coord_t& _size) {

                std::vector<coord_t> result{};
                result.reserve(_open.size());

                const auto end = _open.end();
                for (auto it = _open.begin(); it != end; ++it) {
                    result.emplace_back(utils::to_nd(it->m_index, _size));
                }

                return result;
            }
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
        struct node_data final {

            /** @brief Whether the node is active and should be considered within the search. */
            const bool active;

            /** @brief Index of the node, typically representing its unique identifier within the structure of the search space. */
            const typename params_t::index_type index;

            /** @brief Coordinate of the node in the search space. */
            const typename params_t::coord_type coord;

            /** @brief Computed distance metric (e.g., cost or heuristic) associated with this node relative to its neighbours. */
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

            if constexpr (solver_utils::template is_graph_v<decltype(_params.maze)>) {

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

    public:

        /**
         * @name Constructors
         * @{
         */

        /**
         * @brief Constructor for the `solver` class.
         *
         * @details Initialises a `solver` instance with default values.
         */
        [[nodiscard]] constexpr solver() noexcept = default;
        ~solver() = default;

        solver           (const solver& ) = delete;
        solver           (const solver&&) = delete;
        solver& operator=(const solver& ) = delete;
        solver& operator=(const solver&&) = delete;

        /**
         * @}
         */

        /**
         * @brief Executes the solver with the provided parameters. Parameters are constructed using perfect forwarding.
         *
         * @details Constructs a parameters object using the given arguments, and uses it to invoke the desired solver.
         *
         * @param _args A parameter pack containing the arguments to construct the parameters object.
         *
         * @note You must ensure that the given arguments are valid for constructing the parameters for your intended search.
         *
         * @remarks Calling this will reset the state of `monotonic_pmr`, `heterogeneous_pmr`, and `homogeneous_pmr`, if they have a visible method `reset()`.
         *
         * @warning Aliasing the polymorphic memory resources as other types may hide the `reset()` method from this function.
         *          If you intend for this function to trigger a reset of these parameters, you must provide them as a type where the `reset()` method
         *          is visible.
         *
         * @tparam Args A parameter pack containing the arguments to construct the parameters object.
         * @return A std::vector containing the result of the search. If the search fails, the vector will be empty.
         *
         * @see solve
         */
        template <typename... Args>
        [[maybe_unused, nodiscard]]
#if __cplusplus >= 202003L
        constexpr
#endif // __cplusplus >= 202003L
        static auto solve(Args&&... _args) {
            return solve(std::forward<Args>(_args)...);
        }

        /**
         * @brief Executes the solver with the provided parameters.
         *
         * @param [in] _params The parameters object to solve with.
         *
         * @remarks Calling this will reset the state of `monotonic_pmr`, `heterogeneous_pmr`, and `homogeneous_pmr`, if they have a visible method `reset()`.
         *
         * @warning Aliasing the polymorphic memory resources as other types may hide the `reset()` method from this function.
         *          If you intend for this function to trigger a reset of these parameters, you must provide them as a type where the `reset()` method
         *          is visible.
         *
         * @return A std::vector containing the result of the search. If the search fails, the vector will be empty.
         *
         * @see solve
         */
        [[maybe_unused, nodiscard]]
#if __cplusplus >= 202003L
        constexpr
#endif // __cplusplus >= 202003L
        static auto solve(const params_t& _params) {

            std::vector<typename params_t::coord_type> result{};

            std::exception_ptr exception { nullptr };

            /*
             * Run solver:
             */

            try {
                const auto s = static_cast<typename params_t::index_type>(utils::to_1d(_params.start, _params.size));
                const auto e = static_cast<typename params_t::index_type>(utils::to_1d(_params.end,   _params.size));

                if (_params.maze.contains(s) && _params.maze[s].is_active() &&
                    _params.maze.contains(e) && _params.maze[e].is_active()
                ) {

                    result = s != e ?
                        solver_t<params_t>::invoke(_params) :
                        std::vector<typename params_t::coord_type> { _params.end };
                }
            }
            catch (...) { // NOLINT(*-empty-catch)

                // ReSharper disable once CppDFAUnusedValue
                exception = std::current_exception();
            }

            // Reset memory resources:
            if constexpr (!params_t::no_cleanup::value) {
                try {
                    solver_utils::reset_resources(_params);
                }
                catch (...) { // NOLINT(*-empty-catch)
                    if (exception == nullptr) { exception = std::current_exception(); }
                }
            }

            // Report first exception to occur:
            if (exception != nullptr) {
                std::rethrow_exception(exception);
            }

            return result;
        }

        /**
         * @brief Executes the solver with the provided parameters. Parameters are constructed using perfect forwarding.
         *
         * @details Constructs a parameters object using the given arguments, and uses it to invoke the desired solver.
         *
         * @param _args A parameter pack containing the arguments to construct the parameters object.
         *
         * @note You must ensure that the given arguments are valid for constructing the parameters for your intended search.
         *
         * @remarks Calling this will reset the state of `monotonic_pmr`, `heterogeneous_pmr`, and `homogeneous_pmr`, if they have a visible method `reset()`.
         *
         * @warning Aliasing the polymorphic memory resources as other types may hide the `reset()` method from this function.
         *          If you intend for this function to trigger a reset of these parameters, you must provide them as a type where the `reset()` method
         *          is visible.
         *
         * @tparam Args A parameter pack containing the arguments to construct the parameters object.
         * @return A std::vector containing the result of the search. If the search fails, the vector will be empty.
         *
         * @see solve
         */
        template <typename... Args>
        [[maybe_unused, nodiscard]]
#if __cplusplus >= 202003L
        constexpr
#endif // __cplusplus >= 202003L
        auto operator()(Args&&... _args) {
            return operator()({ std::forward<Args>(_args)... });
        }

        /**
         * @brief Executes the solver with the provided parameters.
         *
         * @param [in] _params The parameters object to solve with.
         *
         * @remarks Calling this will reset the state of `monotonic_pmr`, `heterogeneous_pmr`, and `homogeneous_pmr`, if they have a visible method `reset()`.
         *
         * @warning Aliasing the polymorphic memory resources as other types may hide the `reset()` method from this function.
         *          If you intend for this function to trigger a reset of these parameters, you must provide them as a type where the `reset()` method
         *          is visible.
         *
         * @return A std::vector containing the result of the search. If the search fails, the vector will be empty.
         *
         * @see solve
         */
        [[maybe_unused, nodiscard]]
#if __cplusplus >= 202003L
        constexpr
#endif // __cplusplus >= 202003L
        auto operator()(const params_t& _params) {
            return solve(_params);
        }

        /*
         * Statically-enforce the structure of params_t.
         */

        static_assert(std::is_arithmetic_v<typename params_t::weight_type>,
            "weight_type must be an integral or floating point type.");

        static_assert(std::numeric_limits<typename params_t::weight_type>::is_specialized,
            "weight_type must have defined numeric limits.");

        static_assert(std::is_arithmetic_v<typename params_t::scalar_type>,
            "scalar_type must be an integral or floating point type.");

        static_assert(std::numeric_limits<typename params_t::scalar_type>::is_specialized,
            "scalar_type must have defined numeric limits.");

        static_assert(std::is_integral_v<typename params_t::index_type>,
            "index_type must be an integral type.");

        static_assert(std::numeric_limits<typename params_t::index_type>::is_specialized,
            "index_type must have defined numeric limits.");

        static_assert(solver_utils::template has_subscript_operator_v<typename params_t::coord_type>,
            "coord_type must support array subscript operator [].");

        static_assert(std::is_arithmetic_v<std::decay_t<decltype(std::declval<typename params_t::coord_type>()[0])>>,
            "coord_type must enumerate an arithmetic type.");

        static_assert(std::numeric_limits<typename params_t::scalar_type>::is_specialized,
            "coord_type must enumerate an arithmetic type with defined numeric limits.");

        static_assert(std::is_same_v<typename params_t::lazy_sorting, std::true_type> || std::is_same_v<typename params_t::lazy_sorting, std::false_type>,
            "lazy_sorting must be either std::true_type or std::false_type.");

        static_assert(std::is_same_v<typename params_t::no_cleanup, std::true_type> || std::is_same_v<typename params_t::no_cleanup, std::false_type>,
            "no_cleanup must be either std::true_type or std::false_type.");

        static_assert(std::is_same_v<typename params_t::reverse_equivalence, std::true_type> || std::is_same_v<typename params_t::reverse_equivalence, std::false_type>,
            "no_cleanup must be either std::true_type or std::false_type.");

        // TODO: Further enforcement of params_t structure.
    };

} //chdr::solvers

#endif //CHDR_SOLVER_HPP