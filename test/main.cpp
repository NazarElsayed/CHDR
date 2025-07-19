/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#include <chdr.hpp>
#include <debug.hpp>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <string_view>

#include "core/application.hpp"

namespace test {

    class cli final {

        static constexpr size_t SOLVER             = 1U;
        static constexpr size_t MAZE_WEIGHT        = 2U;
        //static constexpr size_t CLOSED_COMPRESSION = 3U;
        static constexpr size_t X                  = 3U;
        static constexpr size_t Y                  = 4U;
        static constexpr size_t Z                  = 5U;
        static constexpr size_t W                  = 6U;

        static void help() {

            std::cout << "Usage: chdr <command> [arguments]\n"
                      << "\nCommands:\n"
                      << "  <solver> <maze_weight_type> <x>              Process with 1-dimensional coordinate.\n"
                      << "  <solver> <maze_weight_type> <x> <y>          Process with 2-dimensional coordinates.\n"
                      << "  <solver> <maze_weight_type> <x> <y> <z>      Process with 3-dimensional coordinates.\n"
                      << "  <solver> <maze_weight_type> <x> <y> <z> <w>  Process with 4-dimensional coordinates.\n"
                      << "\nSolvers:\n"
                      << "  astar          A*\n"
                      << "  bfs            Breadth-First Search\n"
                      << "  best_first     Best-First Search\n"
                      << "  dfs            Depth-First Search\n"
                      << "  dijkstra       Dijkstra's Algorithm\n"
                      << "  eidastar       Enhanced Iterative-Deepening A*\n"
                      << "  eidbest_first  Enhanced Iterative-Deepening Best-First Search\n"
                      << "  eiddfs         Enhanced Iterative-Deepening Depth-First Search\n"
                      << "  flood          Flood Fill\n"
                      << "  fringe         Fringe Search F*\n"
                      << "  gbest_first    Graveyard Best-First Search\n"
                      << "  gbfs           Graveyard Breadth-First Search\n"
                      << "  gdfs           Graveyard Depth-First Search\n"
                      << "  gjps           Graveyard Jump-Point Search\n"
                      << "  mgstar         Memory-Bounded Graveyard Search\n"
                      << "  smastar        Simplified Memory-Bounded A* Search\n"
                      << "  osmastar       Optimising Simplified Memory-Bounded A* Search\n"
                      << "  gstar          Graveyard Search (G*)\n"
                      << "  idastar        Iterative-Deepening A*\n"
                      << "  idbest_first   Iterative-Deepening Best-First Search\n"
                      << "  iddfs          Iterative-Deepening Depth-First Search\n"
                      << "  jps            Jump-Point Search\n"
                      << "\nMaze Weight Type:\n"
                      << "  bit            Search space represented using 1-bit values.\n"
                      << "  byte           Search space represented using 4-bit values.\n"
                      << "\nExample:\n"
                      << "  chdr astar byte 10\n"
                      << "  chdr astar byte 10 10\n"
                      << "  chdr astar byte 10 10 10\n"
                      << "  chdr astar byte 10 10 10 10\n"
                      << "\nNote: The dimensions must be integers representing coordinates.\n";
        }

        template <template <typename params_t> typename solver_t, typename params_t>
        static int invoke(const params_t& _params) {
            return application::main<solver_t, params_t>(_params);
        }

        template <typename weight_t, typename coord_t, typename scalar_t, typename index_t>
        static chdr::mazes::grid<coord_t, weight_t> make_solvable_random_grid_maze(const coord_t& _start, const coord_t& _end, const coord_t& _size, size_t _seed = 0U, size_t _iterations = std::numeric_limits<size_t>::max()) {

            auto     monotonic = chdr::monotonic_pool();
            auto heterogeneous = chdr::heterogeneous_pool();
            auto   homogeneous = chdr::homogeneous_pool();

            struct params {

                using  weight_type [[maybe_unused]] = weight_t;
                using  scalar_type [[maybe_unused]] = scalar_t;
                using   index_type [[maybe_unused]] =  index_t;
                using   coord_type [[maybe_unused]] =  coord_t;

                using lazy_sorting [[maybe_unused]] = std::true_type;
                using   no_cleanup [[maybe_unused]] = std::false_type;

                const chdr::mazes::grid<coord_t, weight_t>& maze;
                const     coord_type  start;
                const     coord_type  end;
                const     coord_type  size;
                scalar_type  (*h)(const coord_type&, const coord_type&) noexcept;

                decltype(    monotonic)*     monotonic_pmr;
                decltype(heterogeneous)* heterogeneous_pmr;
                decltype(  homogeneous)*   homogeneous_pmr;

                const scalar_type weight       = 1U;
                const      size_t capacity     = 0U;
                const      size_t memoryLimit  = static_cast<size_t>(-1U);
            };

            if (_size.size() >= 2U && _size[0U] > 1U && _size[1U] > 1U) {

                // Generate random 2-KD mazes:
                for (size_t i = 0U; i < _iterations; ++i) {

                    // Seed the random number generator.
                    generator::utils::lcg lcg { _seed + i };

                    constexpr size_t obstacle_threshold = std::numeric_limits<size_t>::max() / 10UL; // 1/10 chance.

                    // Create a random maze using the given threshold for obstacles:
                    std::vector<weight_t> nodes(chdr::utils::product<size_t>(_size));
                    for (size_t j = 1U; j < nodes.size() - 1UL; ++j) {
                        nodes[j] = lcg.operator()() < obstacle_threshold ?
                                       std::numeric_limits<weight_t>::max() :
                                       std::numeric_limits<weight_t>::lowest();
                    }

                    auto result = chdr::mazes::grid<coord_t, weight_t>(_size, nodes);

                    // Check if maze is solvable:
                    if (!(chdr::solvers::solver<chdr::solvers::gbest_first, params>::solve({ result, _start, _end, _size, chdr::heuristics::manhattan_distance<scalar_t, coord_t>, &monotonic, &heterogeneous, &homogeneous }).empty())) {
                        return result; // Return if solvable.
                    }
                }
            }
            else {
                // Edge case for 0D & 1D mazes.
                return chdr::mazes::grid<coord_t, weight_t>(_size, std::vector<weight_t>(chdr::utils::product<size_t>(_size)));
            }

            throw std::runtime_error("ERROR: Could not create a solvable maze!");
        }

        template <typename instanced_solver_t, typename params_t>
        static auto invoke_benchmark(const params_t& _params) {

            auto result = std::numeric_limits<long double>::max();

            try {

                /* TEST SAMPLES */
    #ifndef NDEBUG
                constexpr size_t base_samples = 1UL;
    #else //!NDEBUG
                constexpr size_t base_samples = 100000000UL;
    #endif //!NDEBUG

                size_t test_samples = chdr::utils::max(base_samples / _params.maze.count(), static_cast<size_t>(1U));

                // Graphs are generally more sparse than grids, require fewer samples:
                if constexpr (std::is_same_v<
                    std::decay_t<decltype(_params.maze)>,
                    chdr::mazes::graph<typename params_t::index_type, typename params_t::scalar_type>>
                ) {
                    test_samples = chdr::utils::sqrt(test_samples);
                }

                /* CAPTURE SYSTEM NOISE */
                auto noise_floor_min = std::numeric_limits<long double>::max();
                for (size_t i = 0U; i < test_samples; ++i) {

                    const auto sw_start = std::chrono::high_resolution_clock::now();

                    noise_floor_min = chdr::utils::min(
                        noise_floor_min,
                        std::chrono::duration_cast<std::chrono::duration<long double>>(
                            std::chrono::high_resolution_clock::now() - sw_start
                        ).count()
                    );
                }

                /* TEST ALGORITHM */
                chdr::malloc_consolidate();

                decltype(instanced_solver_t::solve(_params)) path;

                for (size_t i = 0U; i < test_samples; ++i) {

                    const auto sw_start = std::chrono::high_resolution_clock::now();

                    /* INVOKE SOLVE */
                    (void)instanced_solver_t::solve(_params);

                    result = chdr::utils::min(
                        result,
                        std::chrono::duration_cast<std::chrono::duration<long double>>(
                            std::chrono::high_resolution_clock::now() - sw_start
                        ).count()
                    );

                    if (i != test_samples - 1U) {
                        _params.    monotonic_pmr->reset();
                        _params.heterogeneous_pmr->reset();
                        _params.  homogeneous_pmr->reset();

                        std::cout << "resetting\n";
                    }
                }

                result = chdr::utils::max(result - noise_floor_min, std::numeric_limits<long double>::epsilon());
            }
            catch (const std::exception& e) {
                result = std::numeric_limits<long double>::max();

                debug::log(e);
            }

            return result;
        }

        template <typename weight_t, typename coord_t, typename scalar_t, typename index_t>
        static auto run_gppc_benchmarks() {

            auto result = EXIT_FAILURE;

            struct params {

                using weight_type [[maybe_unused]] = weight_t;
                using scalar_type [[maybe_unused]] = scalar_t;
                using  index_type [[maybe_unused]] = index_t;
                using  coord_type [[maybe_unused]] = coord_t;

                using lazy_sorting [[maybe_unused]] = std::false_type;
                using   no_cleanup [[maybe_unused]] = std::true_type;

                const decltype(generator::gppc::map<weight_t, coord_t>::maze)& maze;

                const coord_type start;
                const coord_type end;
                const coord_type size;

                scalar_type (*h)(const coord_type&, const coord_type&) noexcept;

                chdr::    monotonic_pool<>*     monotonic_pmr;
                chdr::heterogeneous_pool<>* heterogeneous_pmr;
                chdr::  homogeneous_pool<>*   homogeneous_pmr;

                const scalar_type weight       = 1U;
                const size_t      capacity     = 0U;
                const size_t      memory_limit = static_cast<size_t>(90U);
            };

            using variant_t = std::variant<
                chdr::solvers::solver<chdr::solvers::        astar, params>,
                chdr::solvers::solver<chdr::solvers::   best_first, params>,
                chdr::solvers::solver<chdr::solvers::          bfs, params>,
                chdr::solvers::solver<chdr::solvers::          dfs, params>,
                chdr::solvers::solver<chdr::solvers::     dijkstra, params>,
                chdr::solvers::solver<chdr::solvers::     eidastar, params>,
                chdr::solvers::solver<chdr::solvers::eidbest_first, params>,
                chdr::solvers::solver<chdr::solvers::       eiddfs, params>,
                chdr::solvers::solver<chdr::solvers::        flood, params>,
                chdr::solvers::solver<chdr::solvers::       fringe, params>,
                chdr::solvers::solver<chdr::solvers::  gbest_first, params>,
                chdr::solvers::solver<chdr::solvers::         gbfs, params>,
                chdr::solvers::solver<chdr::solvers::         gdfs, params>,
                chdr::solvers::solver<chdr::solvers::         gjps, params>,
                chdr::solvers::solver<chdr::solvers::        gstar, params>,
                chdr::solvers::solver<chdr::solvers::      idastar, params>,
                chdr::solvers::solver<chdr::solvers:: idbest_first, params>,
                chdr::solvers::solver<chdr::solvers::        iddfs, params>,
                chdr::solvers::solver<chdr::solvers::          jps, params>,
                chdr::solvers::solver<chdr::solvers::       mgstar, params>,
                chdr::solvers::solver<chdr::solvers::     osmastar, params>,
                chdr::solvers::solver<chdr::solvers::      smastar, params>>;

            struct test {
                const std::string name;
                const variant_t   variant;
            };

            #define MAKE_TEST_VARIANT(name) test { #name, variant_t { std::in_place_type<chdr::solvers::solver<chdr::solvers::name, params>> } }

            const std::array tests {
                MAKE_TEST_VARIANT(        astar),
                // MAKE_TEST_VARIANT(   best_first),
                // MAKE_TEST_VARIANT(          bfs),
                // MAKE_TEST_VARIANT(          dfs),
                // MAKE_TEST_VARIANT(     dijkstra),
                MAKE_TEST_VARIANT(     eidastar),
                // MAKE_TEST_VARIANT(eidbest_first),
                // MAKE_TEST_VARIANT(       eiddfs),
                // MAKE_TEST_VARIANT(        flood),
                // MAKE_TEST_VARIANT(       fringe),
                // MAKE_TEST_VARIANT(  gbest_first),
                // MAKE_TEST_VARIANT(         gbfs),
                // MAKE_TEST_VARIANT(         gdfs),
                // MAKE_TEST_VARIANT(         gjps),
                // MAKE_TEST_VARIANT(        gstar),
                // MAKE_TEST_VARIANT(      idastar),
                // MAKE_TEST_VARIANT( idbest_first),
                // MAKE_TEST_VARIANT(        iddfs),
                MAKE_TEST_VARIANT(          jps),
                MAKE_TEST_VARIANT(       mgstar),
                // MAKE_TEST_VARIANT(     osmastar),
                MAKE_TEST_VARIANT(      smastar)
            };

            #undef MAKE_TEST_VARIANT

            // Find the longest solver's name (for text alignment in readouts):
            size_t longest_solver_name { 0U };
            for (const auto& [name, variant] : tests) {
                longest_solver_name = std::max(longest_solver_name, name.size());
            }

            const auto gppc_dir = std::filesystem::current_path() / "gppc";
            if (std::filesystem::exists(gppc_dir)) {

                std::cout << "~ Running Diagnostics (GPPC) ~\n"
                          << "FORMAT = [SECONDS, BYTES]\n";

                for (const auto& subdir : std::filesystem::directory_iterator(gppc_dir)) {

                    for (const auto& entry : std::filesystem::directory_iterator(subdir)) {

                        try {
                            const auto scenarios_path = std::filesystem::path { entry.path().string() + ".scen" };

                            const auto [map, scenarios] = generator::gppc::generate<weight_t, coord_t, scalar_t>(entry.path(), scenarios_path);

                            std::cout << map.metadata.name << ":\n";

                            for (size_t i = 0U; i != scenarios.size(); ++i) {

                                const auto& scenario = scenarios[i];

                                std::cout << "\tScenario " << (i + 1U) << " (Length: " << scenario.distance << "):\n";

                                for (const auto& [name, variant] : tests) {

                                    // Right-aligned print:
                                    std::cout << "\t\t";
                                    for (size_t j = 0U; j < longest_solver_name - name.size(); ++j) {
                                        std::cout << " ";
                                    }
                                    std::cout << name << ": " << std::flush;

                                    auto monotonic     = chdr::    monotonic_pool();
                                    auto heterogeneous = chdr::heterogeneous_pool();
                                    auto homogeneous   = chdr::  homogeneous_pool();

                                    auto time = std::visit(
                                        [&](const auto& _t) -> long double {
                                            return invoke_benchmark<std::decay_t<decltype(_t)>>( params { map.maze, scenario.start, scenario.end, map.metadata.size, &chdr::heuristics::manhattan_distance<scalar_t, coord_t>, &monotonic, &heterogeneous, &homogeneous });
                                        },
                                        variant
                                    );

                                    size_t peak_memory_usage { 0U };
#ifdef CHDR_DIAGNOSTICS
                                    peak_memory_usage = monotonic.__get_diagnostic_data().peak_allocated +
                                                    heterogeneous.__get_diagnostic_data().peak_allocated +
                                                      homogeneous.__get_diagnostic_data().peak_allocated;
#endif //CHDR_DIAGNOSTICS

                                        monotonic.reset();
                                    heterogeneous.reset();
                                      homogeneous.reset();

                                    std::cout << " " << std::fixed << std::setprecision(9) << time << ", " << peak_memory_usage << "\n";
                                }
                            }
                        }
                        catch (const std::exception& e) {
                            debug::log(e);
                        }
                    }
                }
            }

            return result;
        }

        template <typename weight_t, typename coord_t>
        static int deduce_solver(const std::string_view& _solver, const coord_t& _size) {

            auto result = EXIT_FAILURE;

            using scalar_t = uint32_t;
            using  index_t = uint32_t;

            return run_gppc_benchmarks<weight_t, coord_t, scalar_t, index_t>();

            /* GENERATE MAZE */
            constexpr size_t seed { 0U };

            const coord_t start {};
                  coord_t end;

            for (size_t i = 0U; i < _size.size(); ++i) {
                end[i] = _size[i] - 1U;
            }

            // Empty grid:
            // const std::vector<weight_t> nodes(chdr::utils::product<size_t>(_size), std::numeric_limits<weight_t>::lowest());
            // const auto grid = chdr::mazes::grid<coord_t, weight_t>(_size, nodes);

            // Random grid:
            const auto grid = make_solvable_random_grid_maze<weight_t, coord_t, scalar_t, index_t>(start, end, _size, seed);

            // Maze grid:
            // const auto grid = generator::grid::generate<weight_t>(start, end, _size, 0.0, 0.0, seed);
            //debug::log(_size[0] + _size[1] - 2U);

            const auto& test = grid;
            // auto graph_pool = chdr::heterogeneous_pool(); const auto test = chdr::mazes::graph<index_t, scalar_t>(grid, &graph_pool);
            // const auto test = generator::graph::generate<weight_t, index_t, coord_t, scalar_t>(start, end, size, seed);

            auto     monotonic = chdr::monotonic_pool();
            auto heterogeneous = chdr::heterogeneous_pool();
            auto   homogeneous = chdr::homogeneous_pool();

            struct params {

                using  weight_type [[maybe_unused]] = weight_t;
                using  scalar_type [[maybe_unused]] = scalar_t;
                using   index_type [[maybe_unused]] =  index_t;
                using   coord_type [[maybe_unused]] =  coord_t;

                using lazy_sorting [[maybe_unused]] = std::false_type;
                using   no_cleanup [[maybe_unused]] = std::false_type;

                const decltype(test)& maze;
                const     coord_type  start;
                const     coord_type  end;
                const     coord_type  size;
                         scalar_type  (*h)(const coord_type&, const coord_type&) noexcept;

                decltype(    monotonic)*     monotonic_pmr;
                decltype(heterogeneous)* heterogeneous_pmr;
                decltype(  homogeneous)*   homogeneous_pmr;

                const scalar_type weight       = 1U;
                const      size_t capacity     = 0U;
                const      size_t memory_limit = static_cast<size_t>(90U);
            };

            const params args { test, start, end, _size, chdr::heuristics::manhattan_distance<scalar_t, coord_t>, &monotonic, &heterogeneous, &homogeneous };

                 if (_solver == "astar"        ) { result = invoke<chdr::solvers::        astar, params>(args); }
            else if (_solver == "best_first"   ) { result = invoke<chdr::solvers::   best_first, params>(args); }
            else if (_solver == "bfs"          ) { result = invoke<chdr::solvers::          bfs, params>(args); }
            else if (_solver == "dfs"          ) { result = invoke<chdr::solvers::          dfs, params>(args); }
            else if (_solver == "dijkstra"     ) { result = invoke<chdr::solvers::     dijkstra, params>(args); }
            else if (_solver == "eidastar"     ) { result = invoke<chdr::solvers::     eidastar, params>(args); }
            else if (_solver == "eidbest_first") { result = invoke<chdr::solvers::eidbest_first, params>(args); }
            else if (_solver == "eiddfs"       ) { result = invoke<chdr::solvers::       eiddfs, params>(args); }
            else if (_solver == "flood"        ) { result = invoke<chdr::solvers::        flood, params>(args); }
            else if (_solver == "fringe"       ) { result = invoke<chdr::solvers::       fringe, params>(args); }
            else if (_solver == "gbest_first"  ) { result = invoke<chdr::solvers::  gbest_first, params>(args); }
            else if (_solver == "gbfs"         ) { result = invoke<chdr::solvers::         gbfs, params>(args); }
            else if (_solver == "gdfs"         ) { result = invoke<chdr::solvers::         gdfs, params>(args); }
            else if (_solver == "gjps"         ) { result = invoke<chdr::solvers::         gjps, params>(args); }
            else if (_solver == "gstar"        ) { result = invoke<chdr::solvers::        gstar, params>(args); }
            else if (_solver == "idastar"      ) { result = invoke<chdr::solvers::      idastar, params>(args); }
            else if (_solver == "idbest_first" ) { result = invoke<chdr::solvers:: idbest_first, params>(args); }
            else if (_solver == "iddfs"        ) { result = invoke<chdr::solvers::        iddfs, params>(args); }
            else if (_solver == "jps"          ) { result = invoke<chdr::solvers::          jps, params>(args); }
            else if (_solver == "mgstar"       ) { result = invoke<chdr::solvers::       mgstar, params>(args); }
            else if (_solver == "osmastar"     ) { result = invoke<chdr::solvers::     osmastar, params>(args); }
            else if (_solver == "smastar"      ) { result = invoke<chdr::solvers::      smastar, params>(args); }
            else {
                debug::log("ERROR: Unknown solver \"" + std::string(_solver) + "\"!", error);
            }

            return result;
        }

        template <typename coord_t>
        static int deduce_weight(int _argc, const char* const _argv[], const coord_t& _coord) {

            auto result = EXIT_FAILURE;

            if (_argc > 0 && static_cast<size_t>(_argc) > MAZE_WEIGHT) {

                const std::string maze_format { _argv[MAZE_WEIGHT] };

                     if (maze_format == "bit"   ) { result = deduce_solver<bool>    ( { _argv[SOLVER] }, _coord); }
                else if (maze_format == "byte"  ) { result = deduce_solver<char>    ( { _argv[SOLVER] }, _coord); }
                //else if (maze_format == "int32" ) { result = deduce_solver<int32_t> ( { _argv[SOLVER] }, _coord); }
                //else if (maze_format == "uint32") { result = deduce_solver<uint32_t>( { _argv[SOLVER] }, _coord); }
                //else if (maze_format == "int64" ) { result = deduce_solver<int32_t> ( { _argv[SOLVER] }, _coord); }
                //else if (maze_format == "uint64") { result = deduce_solver<uint32_t>( { _argv[SOLVER] }, _coord); }
                else {
                    debug::log("ERROR: Unknown maze format \"" + std::string(maze_format) + "\"!", error);
                }
            }

            return result;
        }

        static int deduce_coord(int _argc, const char* const _argv[]) {

            auto result = EXIT_FAILURE;

            using index_t = unsigned long;

            //     if (static_cast<size_t>(_argc - 1) == X) { result = deduce_weight(_argc, _argv, chdr::coord<index_t, 1U> { std::stoul(_argv[X]) }); }
            if (static_cast<size_t>(_argc - 1) == Y) { result = deduce_weight(_argc, _argv, chdr::coord<index_t, 2U> { std::stoul(_argv[X]), std::stoul(_argv[Y]) }); }
            //else if (static_cast<size_t>(_argc - 1) == Z) { result = deduce_weight(_argc, _argv, chdr::coord<index_t, 3U> { std::stoul(_argv[X]), std::stoul(_argv[Y]), std::stoul(_argv[Z]) }); }
            //else if (static_cast<size_t>(_argc - 1) == W) { result = deduce_weight(_argc, _argv, chdr::coord<index_t, 4U> { std::stoul(_argv[X]), std::stoul(_argv[Y]), std::stoul(_argv[Z]), std::stoul(_argv[W]) }); }
            else {
                debug::log("ERROR: Invalid Dimensionality!", error);
            }

            return result;
        }

    public:

        static int main(const int _argc, const char* const _argv[]) {

            const auto result = deduce_coord(_argc, _argv);

            if (result != EXIT_SUCCESS) {
                help();
            }

            return result;
        }
    };

}

/**
 * @file main.cpp
 * @brief Global entry point.
 */
int main(const int _argc, const char* const _argv[]) noexcept {

    auto result = EXIT_FAILURE;

    try {
        debug::log("CHDR " CHDR_VERSION);
        debug::log("Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson");
        debug::log("Licensed under CC BY-NC-ND 4.0");

        debug::log("main()", info);

        result = test::cli::main(_argc, _argv);
    }
    catch(...) {} //NOLINT(*-empty-catch)

    return result;
}